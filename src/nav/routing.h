/**
 * routing.h — offline-routing prototype (A* on a small synthetic road grid).
 *
 * A "ROUTE" button on the map enters a crosshair pick mode:
 *   tap 1 = start point, tap 2 = stop point, then a confirm prompt:
 *   "Add path?" -> Yes runs A* and draws the path (timing/memory logged),
 *   No saves the selected start/stop to /sdcard/routing_selection.txt.
 *
 * The graph is a synthetic 8-connected grid generated at init (PSRAM) so the
 * A* engine, heap and memory footprint can be validated on the real board
 * before wiring a real OSM road network. ~GRID_W x GRID_H nodes.
 */
#ifndef ROUTING_H_
#define ROUTING_H_

#include <LovyanGFX.hpp>

/* Build the test grid graph in PSRAM. Call once from setup(). */
void routing_init(void);

/* Boot self-test: run A* across the full grid and log visited/time/mem.
 * Validates the engine on-device without needing the touch UI. */
void routing_selftest(void);

/* Tap handler for the routing state machine. Returns true if the tap was
 * consumed (button / crosshair pick / confirm dialog). */
bool routing_handle_tap(int x, int y);

/* Screen-fixed overlay: ROUTE button, crosshairs, confirm prompt. Called from
 * drawMap() every frame after the HUD (FULL mode only). */
void routing_draw_overlay(LGFX_Sprite &spr);

/* Draw the computed path (if any) onto the north-up world sprite, like the
 * nav route — called when the world is recomposed. */
void routing_draw_world(LGFX_Sprite &world, double refLon, double refLat, int zoom);

#endif // ROUTING_H_
