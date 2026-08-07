/**
 * input_touch.h — FT6336 capacitive-touch input module.
 * Maps raw touch -> screen coords, distinguishes tap / drag / long-press,
 * and dispatches to the map / UI / power modules.
 */
#ifndef INPUT_TOUCH_H_
#define INPUT_TOUCH_H_

void input_touch_poll(void);   /* call every loop */

#endif // INPUT_TOUCH_H_
