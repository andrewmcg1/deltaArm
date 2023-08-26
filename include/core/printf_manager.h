/**
 * <printf_manager.h>
 *
 * @brief   Functions to start and stop the printf mnaager which is a
 * separate thread printing data to the console for debugging.
 *
 * @addtogroup PrintfManager
 * @{
 */

#ifndef __PRINTF_MANAGER__
#define __PRINTF_MANAGER__

#include <flight_mode.h>

/**
 * @brief   Start the printf_manager thread which should be the only thing
 * printing to the screen besides error messages from other threads.
 *
 * Usage: main.c, line 618
 *
 * @return  0 on success, -1 on failure
 */
int printf_init(void);

/**
 * @brief   Waits for the printf manager thread to exit.
 *
 * Usage: main.c, line 655
 *
 * @return  0 on clean exit, -1 on exit time out/force close
 */
int printf_cleanup(void);

/**
 * @brief   Only used by printf_manager right now, but could be useful
 * elsewhere.
 *
 * Usage: printf_manager.c, line 273
 *
 * @param[in]  mode  The flight mode
 *
 * @return     0 on success or -1 on error
 */
int print_flight_mode(flight_mode_t mode);

#endif /* __PRINTF_MANAGER__ */

/* @} end group PrintfManager */
