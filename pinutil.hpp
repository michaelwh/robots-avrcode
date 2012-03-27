/*
 * pinutil.hpp
 *
 *  Created on: Mar 26, 2012
 *      Author: mh23g08
 */

#ifndef PINUTIL_HPP_
#define PINUTIL_HPP_

#define SET_BIT(p,n) ((p) |= (1 << (n)))
#define CLR_BIT(p,n) ((p) &= ~((1) << (n)))

#define CHECK_BIT(p,n) ((p) & (1<<(n)))


#endif /* PINUTIL_HPP_ */
