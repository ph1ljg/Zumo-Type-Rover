/*
 * Compiler.h
 *
 * Created: 18/05/2020 11:52:00
 *  Author: philg
 */ 


#ifndef COMPILER_H_
#define COMPILER_H_

//========================================================================
//  Determine if an expression evaluates to a constant value.
//  Param[in] exp Any expression
// return true if \a exp is constant, false otherwise.

#if (defined __GNUC__) || (defined __CC_ARM)
#   define is_constant(exp)       __builtin_constant_p(exp)
#else
#   define is_constant(exp)       (0)
#endif




#endif /* COMPILER_H_ */