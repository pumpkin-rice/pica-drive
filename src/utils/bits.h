/**
 * @file bits.h
 * @author Pumpkin Rice (pumpkin_rice@163.com)
 * @brief 
 * @version 0.1
 * @date 2026-04-19
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _BITS_H_
#define _BITS_H_

#define bit(_x)                     (1ul << (_x))
#define set_bit(_num, _bit)         ((_num) |= (_bit))
#define clear_bit(_num, _bit)       ((_num) &= ~(_bit))
#define is_bit_set(_num, _bit)      ((_bit) == ((_num) & (_bit)))
#define is_bits_set(_num, _bits)    ((_bits) == ((_num) & (_bits)))

#define bit_mask(_x)                (~(1ul << (_x)))

#define gen_mask(_h, _l)            ((bit(_h) - 1) & ~(bit(_l) - 1))
#define bits_range(_h, _l)          (gen_mask(_h, _l))

#define shift_left(_num, _l)        ((_num) << (_l))
#define shift_right(_num, _r)       ((_num) >> (_r))

#define shift_left_loop(_num, _l, _bits)     (((_num) << (_l)) | ((_num) >> ((_bits) - (_l))))
#define shift_left_loop8(_num, _l)           shift_left_loop(_num, _l, 8)
#define shift_left_loop16(_num, _l)          shift_left_loop(_num, _l, 16)
#define shift_left_loop32(_num, _l)          shift_left_loop(_num, _l, 32)

#define shift_right_loop(_num, _r, _bits)     (((_num) >> (_r)) | ((_num) << ((_bits) - (_r))))
#define shift_right_loop8(_num, _r)           shift_left_loop(_num, _r, 8)
#define shift_right_loop16(_num, _r)          shift_left_loop(_num, _r, 16)
#define shift_right_loop32(_num, _r)          shift_left_loop(_num, _r, 32)

#endif /* !_BITS_H_ */
