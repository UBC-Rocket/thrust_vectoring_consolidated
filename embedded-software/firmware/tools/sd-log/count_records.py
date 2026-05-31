import json
from collections import Counter

counts = Counter()
with open("sd-log/logs/flight.jsonl") as f:
    for line in f:
        r = json.loads(line)
        name = r.get("record_name", f"unknown_0x{r['record_id']:02x}")
        counts[name] += 1

for name, n in sorted(counts.items()):
    print(f"{name:30s} {n:8d}")
