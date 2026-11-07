# Conferma di prontezza di un drone, pubblicata mentre attende il via.
#
# Sostituisce due meccanismi fragili nel banco di prova: la lettura dei log con
# grep (un solo drone che ripeteva "wait for trigger" 62 volte veniva contato
# come tre) e il riscaldamento a tempo fisso, che approssimava la condizione
# "la mappa e' pronta" con un cronometro invece di verificarla.
#
# Pubblicata con QoS transient_local: chi si iscrive dopo riceve comunque
# l'ultimo stato, quindi il coordinatore puo' partire in qualsiasi momento.
int32 drone_id

# Precondizioni, ognuna osservabile separatamente per sapere DOVE si e' fermato
# un drone che non parte.
bool odom_ok           # l'odometria arriva
bool map_ok            # la mappa ha integrato almeno una scansione
int32 num_frontiers    # frontiere viste: >0 significa che c'e' qualcosa da esplorare

# Vero solo quando tutte le precondizioni sono soddisfatte.
bool ready

# Motivo leggibile quando ready e' falso, per non doverlo dedurre dai log.
string detail

float64 stamp
