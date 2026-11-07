# Periodic state broadcast of one UAV to its communication neighbours.
int32 drone_id

int32[] grid_ids
geometry_msgs/Point[] center_points
int32[] center_ids
float64 recent_attempt_time
float64 stamp

float32[] pos
float32[] vel
float32 yaw

# SCIAME, fase di visita (ottobre 2026). Fase della missione: 0 esplorazione a
# frontiere, 1 completamento (visita dello spazio percorribile / foto), 2 rientro o
# missione finita. meta_visita = zona che il drone sta andando a visitare (valida se
# ha_meta_visita): gli altri non la scelgono e si spartiscono le restanti.
uint8 fase
bool ha_meta_visita
geometry_msgs/Point meta_visita
