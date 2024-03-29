/*
 * TMC2300_Constants.h
 *
 *  Created on: 16.01.2019
 *      Author: LK
 *
 *  Modified on: 17.03.2023
 *      Author: SJFOM
 *      Brief: Added Version field & comment
 */

#ifndef TMC_IC_TMC2300_TMC2300_CONSTANTS_H_
#define TMC_IC_TMC2300_TMC2300_CONSTANTS_H_

#define TMC2300_MOTORS           1
#define TMC2300_REGISTER_COUNT   TMC_REGISTER_COUNT
#define TMC2300_WRITE_BIT        TMC_WRITE_BIT
#define TMC2300_ADDRESS_MASK     TMC_ADDRESS_MASK
#define TMC2300_MAX_VELOCITY     s32_MAX
#define TMC2300_MAX_ACCELERATION u24_MAX

 // 0x40=first version of the IC. Identical numbers mean full digital compatibility.
#define TMC2300_VERSION_COMPATIBLE 0x40

#endif /* TMC_IC_TMC2300_TMC2300_CONSTANTS_H_ */