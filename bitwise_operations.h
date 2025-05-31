#ifndef BITWISE_OPERATIONS_H
#define BITWISE_OPERATIONS_H

#define SET_BIT(REG, BIT)     ((REG) |= (1U << (BIT)))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(1U << (BIT)))

#define TOGGLE_BIT(REG, BIT)  ((REG) ^= (1U << (BIT)))

#define GET_BIT(REG, BIT)     (((REG) >> (BIT)) & 1U)

#endif // BITWISE_OPERATIONS_H
