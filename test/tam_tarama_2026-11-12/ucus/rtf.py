import time, sys
from gz.transport13 import Node
from gz.msgs10.world_stats_pb2 import WorldStatistics
d=[]; n=Node()
n.subscribe(WorldStatistics,"/world/default/stats",lambda m:d.append((m.sim_time.sec+m.sim_time.nsec*1e-9,m.real_time.sec+m.real_time.nsec*1e-9)))
time.sleep(float(sys.argv[1]) if len(sys.argv)>1 else 20)
print("veri yok" if len(d)<2 else f"etkin RTF = {(d[-1][0]-d[0][0])/(d[-1][1]-d[0][1]):.3f}")
