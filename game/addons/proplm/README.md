# proplm

A small Garry's Mod addon that makes props look right in maps built with the proplm compile option.

Normally a prop's baked lighting is stored on the same texture coordinates as its paint. When a model reuses parts of
its texture (mirrored or tiled areas), the baked lighting comes out wrong. Maps compiled with the proplm option give
those props a second set of coordinates just for lighting. This addon draws them that way.

## Install

Copy the `proplm` folder into `GarrysMod/garrysmod/addons/`, so that you end up with
`GarrysMod/garrysmod/addons/proplm/lua/...`. Restart the game. There is nothing to configure.

## What you need

A map compiled with the proplm option, with the lighting step (VRAD) run normally. On any other map the addon does
nothing and prints nothing.

## Using it

It works on its own. Two console commands are there if you want to look:

- `proplm_status` shows what happened to each converted prop in the current map.
- `proplm_reload` rebuilds everything.

Two settings: `proplm_enable 0` turns the addon off (props go back to their normal look), `proplm_debug 1` prints
details. `proplm_debug 2` followed by `proplm_reload` shows the baked lighting alone on white props.

## Good to know

- Props with a non-default skin, a changed size or a colour tint are left as normal props.
- Converted props do not cast shadows on the world in the baked lighting, and the baked lighting is not updated if a
  prop moves.
