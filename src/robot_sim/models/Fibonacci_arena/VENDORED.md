# Vendored: Fibonacci_arena

- Source: git@github.com:Cinar13/Fibonacci-Autonomous-Map.git, commit `fa04600` (`meshes/Part1.stl`).
  The team's own race map; `fibonacci.sdf` there is the bare model this directory wraps.
- `meshes/Part1.stl` is unmodified (binary STL in millimetres, scaled by 0.001 in `model.sdf`).
  The Solid Edge source `Part1.par` is not vendored; edit the map there and copy the exported STL here.
- Geometry: 2.49 x 2.10 m box, 0.25 m high walls; a U island (open to +y, arms at x = +-0.62 m from
  y = -0.42 to 0.42 m) and a divider from the top wall down to y = 0.21 m. Corridors are about 0.6 m wide.
- Loaded by `worlds/fibonacci_harmonic.sdf` via `model://Fibonacci_arena`.
