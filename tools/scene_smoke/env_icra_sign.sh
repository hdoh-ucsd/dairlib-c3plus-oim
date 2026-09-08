# Faithful icra_sign planner env (anything_icra_c_v5 recipe): C-glyph
# footprint, glyph top height, and the 7 exact 10-vertex hulls (indices 0-6);
# index 7 (robot base) stays a disc.
SAMPLING_C3_OBJECT_FOOTPRINT=c_glyph
SAMPLING_C3_OBS_TOP_Z=0.046
SAMPLING_C3_OBS_POLYS="$(cat "$(dirname "${BASH_SOURCE[0]}")/icra_obs_polys_env.txt")"
