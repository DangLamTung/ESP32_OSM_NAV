# PLAN — Draw navigation on the offline map (osm_idf)

Goal: render live turn-by-turn navigation on the ILI9341 map, fed by the phone
over BLE. The phone computes the route (offline on its side) and streams compact
XML; the board only has to **draw** it over the offline Carto tiles.

---

## 0. What already works (don't rebuild this)

- **BLE nav input** — `ble/ble_nav.cpp` receives over GATT:
  - `<route z="15"><p lat=".." lon=".."/>...</route>` → `NavRoute` (polyline)
  - `<nav d="85" m="left" s="Nguyen Hue"/>` → `NavManeuver` (next turn)
  - `<pos lat=".." lon=".." spd="34" hdg="312"/>` → `NavPos` (live position)
  - `nav*Dirty()` flags + `navGetRoute/Maneuver/Pos()` getters.
- **Overlay drawing** — `ui/ui_controls.cpp` `ui_draw_nav_overlay()` already draws:
  - thick blue **route polyline**, red **position dot**, **maneuver HUD** box
    (arrow + distance + street name, Vietnamese→ASCII).
  - called from `main.cpp` `drawMap()` between `map_fetch()` and `map_push()`.
- **Auto-follow** — `main.cpp loop()` recenters when the position drifts >40 px.

So Phase 1 is mostly polish; the real wins are Phase 2 + 4.

---

## 1. Route drawing — polish & robustness

- **Casing**: draw the route as dark outline (e.g. 5px #333) then bright core
  (e.g. 3px #1a73e8) so it's visible on any tile color.
- **Viewport clip**: skip points fully outside the 320x240 view; don't project
  all 512 points every frame — early-out on the tile span.
- **Turn arrows**: draw small chevrons along the route at maneuver points so the
  path reads at a glance (project each maneuver lat/lon, place a rotated `>`).
- **Start / end markers**: green circle at origin, checkered/red flag at the
  destination.
- **Cache projected points** keyed by (zoom, center tile) so pan/zoom redraws
  reuse work instead of re-projecting the whole polyline.

## 2. Live position + follow — the UX that matters

- **Heading marker**: draw the position as a **rotated triangle/arrow** using
  `NavPos.hdg` (currently just a dot).
- **Follow modes** (tap to cycle):
  - *Center*: recenter on the vehicle (current behavior).
  - *Look-ahead*: keep the vehicle in the lower third of the screen so more
    road ahead is visible (like real nav) — offset the view center by ~60 px
    opposite the heading.
- **Smooth follow**: instead of jumping when the 40 px threshold trips, lerp the
  center toward the position a few % per frame (feels like tracking, not
  snapping).
- **Distance-to-turn countdown** in the HUD (already have `man->dist`).

## 3. Maneuver HUD — enhance (cheap)

- Color per maneuver type (left=green, right=green, u-turn=amber, arrive=red).
- Show the street **after** the turn if the phone sends it (`<nav s="..">` is
  current street; can add `t="next street"`).
- Keep the ASCII Vietnamese strip (already implemented).

## 4. Integration & performance — the important fix

- **Do NOT re-fetch tiles on nav updates.** Today `navPosDirty()` sets
  `s_mapDirty` → full `map_fetch()` (re-decodes 4–9 tiles every position ping
  ~1 Hz) → slow + flickery while navigating.
  - Fix: on route/pos/maneuver dirty, **re-push the cached sprite**
    (`map_push()`) then draw the overlay on top — no tile re-decode.
  - Only a genuine pan/zoom forces a tile re-fetch.
- Split `main.cpp` redraw logic: `dirty_map` (tiles) vs `dirty_overlay`
  (nav/UI on top of the existing sprite).
- Keep all nav buffers in PSRAM; no allocation inside the draw loop.

## 5. Testing

- Feed test XML from a PC BLE client (or the phone navbridge app):
  - static route → verify polyline + markers + arrows
  - live `<pos>` stream → verify smooth follow + heading arrow
  - `<nav>` updates → verify HUD changes + countdown
  - pan/zoom during nav → route stays glued to the map (overlay redraw, no tile
    flicker)
- Watch `map drawn (x ms)` in serial: during nav it should be **overlay-only**
  (~ms), not a full tile fetch (~100 ms+).

---

## Suggested commit order

1. Overlay-only redraw path (Phase 4) — biggest win, unblocks smooth nav.
2. Route casing + clipping + arrows + markers (Phase 1).
3. Heading marker + look-ahead + smooth follow (Phase 2).
4. HUD polish (Phase 3).
