# Agents Guide

## Project Overview

This is a static 2D parking simulator. The app renders a parking lot on a canvas, lets users toggle parked cars as obstacles, and plans forward or reverse parking maneuvers into a selected spot.

## File Layout

- `index.html`: Markup only. Keep document structure and controls here.
- `styles.css`: All visual styling, responsive layout, and theme variables.
- `app.js`: Simulator state, rendering, path planning, event handlers, and animation.
- `scripts/smoke-planner.mjs`: Node-based smoke test for representative planner scenarios.
- `README.md`: Basic run instructions.

## Local Commands

- `npm start`: Serve the project at `http://localhost:4173`.
- `npm test`: Run planner smoke scenarios.

## Development Notes

- Keep the app dependency-free unless a new dependency clearly pays for itself.
- Preserve the existing element IDs when changing markup; `app.js` and the smoke test depend on them.
- Put new presentation rules in `styles.css`, not inline HTML attributes.
- Put simulator behavior in `app.js`, keeping `index.html` free of inline scripts.
- After planner or collision changes, run `npm test`.
