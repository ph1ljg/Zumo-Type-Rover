/* 
* CTimers.cpp
*
* Created: 11/04/2021 10:02:04
* Author: philg
*/

#include "Includes.h"
#include <assert.h>
#include "CTimers.h"

// default constructor
CTimers::CTimers()
{
} //CTimers

// default destructor
CTimers::~CTimers()
{
} //~CTimers
/**
 * \brief Configures a Timer Counter Channel
 *
 * Configures a Timer Counter to operate in the given mode. Timer is stopped
 * after configuration and must be restarted with TC_Start(). All the
 * interrupts of the timer are also disabled.
 *
 * \param pTc  Pointer to a Tc instance.
 * \param channel Channel number.
 * \param mode  Operating mode (TC_CMR value).
 */
void TC_Configure( Tc1 *pTc, uint32_t dwChannel, uint32_t dwMode )
{
    TcChannel1* pTcCh ;

    assert( dwChannel < (sizeof( pTc->TC_CHANNEL )/sizeof( pTc->TC_CHANNEL[0] )) ) ;
    pTcCh = pTc->TC_CHANNEL+dwChannel ;

    /*  Disable TC clock */
    pTcCh->TC_CCR = TC_CCR_CLKDIS ;

    /*  Disable interrupts */
    pTcCh->TC_IDR = 0xFFFFFFFF ;

    /*  Clear status register */
    pTcCh->TC_SR ;

    /*  Set mode */
    pTcCh->TC_CMR = dwMode ;
}

/**
 * \brief Reset and Start the TC Channel
 *
 * Enables the timer clock and performs a software reset to start the counting.
 *
 * \param pTc  Pointer to a Tc instance.
 * \param dwChannel Channel number.
 */
extern void TC_Start( Tc1 *pTc, uint32_t dwChannel )
{
    TcChannel1* pTcCh ;

    assert( dwChannel < (sizeof( pTc->TC_CHANNEL )/sizeof( pTc->TC_CHANNEL[0] )) ) ;

    pTcCh = pTc->TC_CHANNEL+dwChannel ;
    pTcCh->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG ;
}

/**
 * \brief Stop TC Channel
 *
 * Disables the timer clock, stopping the counting.
 *
 * \param pTc     Pointer to a Tc instance.
 * \param dwChannel Channel number.
 */
extern void TC_Stop(Tc1 *pTc, uint32_t dwChannel )
{
    TcChannel1* pTcCh ;

    assert( dwChannel < (sizeof( pTc->TC_CHANNEL )/sizeof( pTc->TC_CHANNEL[0] )) ) ;

    pTcCh = pTc->TC_CHANNEL+dwChannel ;
    pTcCh->TC_CCR = TC_CCR_CLKDIS ;
}

/**
 * \brief Find best MCK divisor
 *
 * Finds the best MCK divisor given the timer frequency and MCK. The result
 * is guaranteed to satisfy the following equation:
 * \code
 *   (MCK / (DIV * 65536)) <= freq <= (MCK / DIV)
 * \endcode
 * with DIV being the highest possible value.
 *
 * \param dwFreq  Desired timer frequency.
 * \param dwMCk  Master clock frequency.
 * \param dwDiv  Divisor value.
 * \param dwTcClks  TCCLKS field value for divisor.
 * \param dwBoardMCK  Board clock frequency.
 *
 * \return 1 if a proper divisor has been found, otherwise 0.
 */
extern uint32_t TC_FindMckDivisor( uint32_t dwFreq, uint32_t dwMCk, uint32_t *dwDiv, uint32_t *dwTcClks, uint32_t dwBoardMCK )
{
    const uint32_t adwDivisors[5] = { 2, 8, 32, 128, dwBoardMCK / 32768 } ;

    uint32_t dwIndex = 0 ;

    /*  Satisfy lower bound */
    while ( dwFreq < ((dwMCk / adwDivisors[dwIndex]) / 65536) )
    {
        dwIndex++ ;

        /*  If no divisor can be found, return 0 */
        if ( dwIndex == (sizeof( adwDivisors )/sizeof( adwDivisors[0] ))  )
        {
            return 0 ;
        }
    }

    /*  Try to maximize DIV while satisfying upper bound */
    while ( dwIndex < 4 )
    {

        if ( dwFreq > (dwMCk / adwDivisors[dwIndex + 1]) )
        {
            break ;
        }
        dwIndex++ ;
    }

    /*  Store results */
    if ( dwDiv )
    {
        *dwDiv = adwDivisors[dwIndex] ;
    }
    if ( dwTcClks )
    {
        *dwTcClks = dwIndex ;
    }

    return 1 ;
}

/**
 * \brief Read Timer Counter Counter Value on the selected TC & channel.
 *
 * \param p_tc Pointer to a TC instance.
 * \param ul_channel Channel to configure.
 *
 * \return RC value.
 */
uint32_t TC_ReadCV(Tc1 *p_tc, uint32_t ul_channel){
	return p_tc->TC_CHANNEL[ul_channel].TC_CV;
}

/**
 * \brief Get current status on the selected channel.
 *
 * \param p_tc Pointer to a TC instance.
 * \param ul_channel Channel to configure.
 *
 * \return The current TC status.
 */
uint32_t TC_GetStatus(Tc1 *p_tc, uint32_t ul_channel)
{
	TcChannel1 *tc_channel;

	tc_channel = p_tc->TC_CHANNEL + ul_channel;
	return tc_channel->TC_SR;
}

/**
 * \brief Set RA on the selected channel.
 *
 * \param tc Pointer to a TC instance.
 * \param chan Channel to configure.
 * \param v New value for RA.
 */
void TC_SetRA(Tc1 *tc, uint32_t chan, uint32_t v) {
	tc->TC_CHANNEL[chan].TC_RA = v;
}

/**
 * \brief Set RB on the selected channel.
 *
 * \param tc Pointer to a TC instance.
 * \param chan Channel to configure.
 * \param v New value for RB.
 */
void TC_SetRB(Tc1 *tc, uint32_t chan, uint32_t v) {
	tc->TC_CHANNEL[chan].TC_RB = v;
}

/**
 * \brief Set RC on the selected channel.
 *
 * \param tc Pointer to a TC instance.
 * \param chan Channel to configure.
 * \param v New value for RC.
 */
void TC_SetRC(Tc1 *tc, uint32_t chan, uint32_t v) {
	tc->TC_CHANNEL[chan].TC_RC = v;
}

