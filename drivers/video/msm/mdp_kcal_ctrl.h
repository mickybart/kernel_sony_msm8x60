
/*
 * Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 * Copyright (c) 2013, LGE Inc. All rights reserved
 * Copyright (c) 2014, savoca <adeddo27@gmail.com>
 * Copyright (c) 2014, Paul Reioux <reioux@gmail.com>
 * Copyright (c) 2016 Michaël Serpieri <mickybart@pygoscelis.org>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MDP_KCAL_CTRL_H
#define __MDP_KCAL_CTRL_H

#define NUM_QLUT 256
#define MAX_KCAL_V (NUM_QLUT-1)

struct kcal_lut_data {
	int red;
	int green;
	int blue;
	int minimum;
	int enable;
};

void mdp_pp_kcal_enable(bool enable);
void mdp_pp_kcal_update(int kr, int kg, int kb);
#endif
