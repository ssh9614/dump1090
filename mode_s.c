// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// mode_s.c: Mode S message decoding.
//
// Copyright (c) 2014-2016 Oliver Jowett <oliver@mutability.co.uk>
//
// This file is free software: you may copy, redistribute and/or modify it  
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your  
// option) any later version.  
//
// This file is distributed in the hope that it will be useful, but  
// WITHOUT ANY WARRANTY; without even the implied warranty of  
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU  
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License  
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// This file incorporates work covered by the following copyright and  
// permission notice:
//
//   Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//    *  Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//    *  Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#include "dump1090.h"

/* for PRIX64 */
#include <inttypes.h>

#include <assert.h>

//
// ===================== Mode S detection and decoding  ===================
//
//
//

//=========================================================================
//
// Given the Downlink Format (DF) of the message, return the message length in bits.
//
// All known DF's 16 or greater are long. All known DF's 15 or less are short. 
// There are lots of unused codes in both category, so we can assume ICAO will stick to 
// these rules, meaning that the most significant bit of the DF indicates the length.
//
int modesMessageLenByType(int type) {
    return (type & 0x10) ? MODES_LONG_MSG_BITS : MODES_SHORT_MSG_BITS ;
}

// Correct a decoded native-endian Address Announced field
// (from bits 8-31) if it is affected by the given error
// syndrome. Updates *addr and returns >0 if changed, 0 if
// it was unaffected.
static int correct_aa_field(uint32_t *addr, struct errorinfo *ei) 
{
    int i;
    int addr_errors = 0;

    if (!ei)
        return 0;

    for (i = 0; i < ei->errors; ++i) {
        if (ei->bit[i] >= 8 && ei->bit[i] <= 31) {
            *addr ^= 1 << (31 - ei->bit[i]);
            ++addr_errors;
        }
    }

    return addr_errors;
}

// The first bit (MSB of the first byte) is numbered 1, for consistency
// with how the specs number them.

// Extract one bit from a message.
static inline  __attribute__((always_inline)) unsigned getbit(unsigned char *data, unsigned bitnum)
{
    unsigned bi = bitnum - 1;
    unsigned by = bi >> 3;
    unsigned mask = 1 << (7 - (bi & 7));

    return (data[by] & mask) != 0;
}

// Extract some bits (firstbit .. lastbit inclusive) from a message.
static inline  __attribute__((always_inline)) unsigned getbits(unsigned char *data, unsigned firstbit, unsigned lastbit)
{
    unsigned fbi = firstbit - 1;
    unsigned lbi = lastbit - 1;
    unsigned nbi = (lastbit - firstbit + 1);

    unsigned fby = fbi >> 3;
    unsigned lby = lbi >> 3;
    unsigned nby = (lby - fby) + 1;

    unsigned shift = 7 - (lbi & 7);
    unsigned topmask = 0xFF >> (fbi & 7);

    assert (fbi <= lbi);
    assert (nbi <= 32);
    assert (nby <= 5);

    if (nby == 5) {
        return
            ((data[fby] & topmask) << (32 - shift)) |
            (data[fby + 1] << (24 - shift)) |
            (data[fby + 2] << (16 - shift)) |
            (data[fby + 3] << (8 - shift)) |
            (data[fby + 4] >> shift);
    } else if (nby == 4) {
        return
            ((data[fby] & topmask) << (24 - shift)) |
            (data[fby + 1] << (16 - shift)) |
            (data[fby + 2] << (8 - shift)) |
            (data[fby + 3] >> shift);
    } else if (nby == 3) {
        return
            ((data[fby] & topmask) << (16 - shift)) |
            (data[fby + 1] << (8 - shift)) |
            (data[fby + 2] >> shift);
    } else if (nby == 2) {
        return
            ((data[fby] & topmask) << (8 - shift)) |
            (data[fby + 1] >> shift);
    } else if (nby == 1) {
        return
            (data[fby] & topmask) >> shift;
    } else {
        return 0;
    }
}

// Score how plausible this ModeS message looks.
// The more positive, the more reliable the message is

// 1000: DF 0/4/5/16/24 with a CRC-derived address matching a known aircraft

// 1800: DF17/18 with good CRC and an address matching a known aircraft
// 1400: DF17/18 with good CRC and an address not matching a known aircraft
//  900: DF17/18 with 1-bit error and an address matching a known aircraft
//  700: DF17/18 with 1-bit error and an address not matching a known aircraft
//  450: DF17/18 with 2-bit error and an address matching a known aircraft
//  350: DF17/18 with 2-bit error and an address not matching a known aircraft

//   -1: message might be valid, but we couldn't validate the CRC against a known ICAO
//   -2: bad message or unrepairable CRC error

static unsigned char all_zeros[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int scoreModesMessage(unsigned char *msg, int validbits)
{
    int msgtype, msgbits, crc;
    uint32_t addr;
    struct errorinfo *ei;

    if (validbits < 56)
        return -2;

    msgtype = getbits(msg, 1, 5); // Downlink Format
    msgbits = modesMessageLenByType(msgtype);

    if (validbits < msgbits)
        return -2;

    if (!memcmp(all_zeros, msg, msgbits/8))
        return -2;

    crc = modesChecksum(msg, msgbits);

    switch (msgtype) {
    case 4: // surveillance, altitude reply
    case 5: // surveillance, altitude reply
        return icaoFilterTest(crc) ? 1000 : -1;
      
    case 17:   // Extended squitter
    case 18:   // Extended squitter/non-transponder
        ei = modesChecksumDiagnose(crc, msgbits);
        if (!ei)
            return -2; // can't correct errors

        // fix any errors in the address field
        addr = getbits(msg, 9, 32);
        correct_aa_field(&addr, ei);        

        if (icaoFilterTest(addr))
            return 1800 / (ei->errors+1);
        else
            return 1400 / (ei->errors+1);

    default:
        // unknown message type
        return -2;
    }
}

//
//=========================================================================
//
// Decode a raw Mode S message demodulated as a stream of bytes by detectModeS(), 
// and split it into fields populating a modesMessage structure.
//

// return 0 if all OK
//   -1: message might be valid, but we couldn't validate the CRC against a known ICAO
//   -2: bad message or unrepairable CRC error

int decodeModesMessage(struct modesMessage *mm, unsigned char *msg)
{
    // Work on our local copy.
    memcpy(mm->msg, msg, MODES_LONG_MSG_BYTES);
    // Preserve the original uncorrected copy for later forwarding
    memcpy(mm->verbatim, msg, MODES_LONG_MSG_BYTES);
    msg = mm->msg;

    // don't accept all-zeros messages
    if (!memcmp(all_zeros, msg, 7))
        return -2;

    // Get the message type ASAP as other operations depend on this
    mm->msgtype         = getbits(msg, 1, 5); // Downlink Format
    mm->msgbits         = modesMessageLenByType(mm->msgtype);
    mm->crc             = modesChecksum(msg, mm->msgbits);
    mm->correctedbits   = 0;
    mm->addr            = 0;

    // Do checksum work and set fields that depend on the CRC
    switch (mm->msgtype) {
    case 4: // surveillance, altitude reply
    case 5: // surveillance, altitude reply
        // These message types use Address/Parity, i.e. our CRC syndrome is the sender's ICAO address.
        // We can't tell if the CRC is correct or not as we don't know the correct address.
        // Accept the message if it appears to be from a previously-seen aircraft
        if (!icaoFilterTest(mm->crc)) {
           return -1;
        }
        mm->addr = mm->crc;
        break;

    case 17:   // Extended squitter
    case 18: { // Extended squitter/non-transponder
        struct errorinfo *ei;
        int addr1, addr2;

        // These message types use Parity/Interrogator

        if (mm->crc != 0) {
            ei = modesChecksumDiagnose(mm->crc, mm->msgbits);
            if (!ei) {
                return -2; // couldn't fix it
            }

            addr1 = getbits(msg, 9, 32);
            mm->correctedbits = ei->errors;
            modesChecksumFix(msg, ei);
            addr2 = getbits(msg, 9, 32);
        
            // we are conservative here: only accept corrected messages that
            // match an existing aircraft.
            if (addr1 != addr2 && !icaoFilterTest(addr2)) {
                return -1;
            }
        }

        break;
    }

    default:
        // All other message types, we don't know how to handle their CRCs, give up
        return -2;
    }

    // AA (Address announced)
    if (mm->msgtype == 11 || mm->msgtype == 17 || mm->msgtype == 18) {
        mm->AA = mm->addr = getbits(msg, 9, 32);
    }      

    if (!mm->correctedbits && (mm->msgtype == 17 || mm->msgtype == 18)) {
        // No CRC errors seen, We probably have the right address.

        // NB this is the only place that adds addresses!
        icaoFilterAdd(mm->addr);
    }

    // all done
    return 0;
}

//
//=========================================================================
//
// When a new message is available, because it was decoded from the RTL device, 
// file, or received in the TCP input port, or any other way we can receive a 
// decoded message, we call this function in order to use the message.
//
// Basically this function passes a raw message to the upper layers for further
// processing and visualization
//
void useModesMessage(struct modesMessage *mm) {

    // Feed output clients.
    modesQueueOutput(mm);
}

//
// ===================== Mode S detection and decoding  ===================
//
