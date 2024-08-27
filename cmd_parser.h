/* stuff for receiving commands etc */

/* (c) copyright fenugrec 2016
 * GPLv3
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "stypes.h"


void cmd_init(u8 brrdiv);
//void can_init(unsigned int param_1,short param_2,unsigned short param_3,unsigned short param_4,unsigned int param_5,char param_6,unsigned int mailbox);

static void can_tx8bytes(const u8 *buf);
static void can_idle(unsigned us);

void cmd_loop(void);

void can_cmd_loop(void);
