import os, sys, unicodedata

check_files = [
    'ground_station/app.py',
    'ground_station/theme.py',
    'ground_station/graphs/plot.py',
    'ground_station/pages/overview.py',
    'ground_station/pages/vehicle.py',
    'ground_station/pages/command.py',
    'ground_station/pages/com.py',
    'ground_station/pages/experiment.py',
    'ground_station/widgets/health_strip.py',
    'ground_station/experiment/profile_plot.py',
    'main.py'
]

for p in check_files:
    if os.path.exists(p):
        with open(p, 'r', encoding='utf-8') as fp:
            for idx, line in enumerate(fp, 1):
                clean = line.strip()
                if clean.startswith('#') or clean.startswith('"""'):
                    continue
                for ch in line:
                    cp = ord(ch)
                    cat = unicodedata.category(ch)
                    if cat in ['So', 'Sk'] and cp > 127:
                        name = unicodedata.name(ch, '')
                        if 'BOX DRAWINGS' in name:
                            continue
                        print(f"{p}:{idx} -> U+{cp:04X} {ch} ({name}) in: {clean[:80]}")
