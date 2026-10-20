import os, unicodedata

allowed_names = {
    'DEGREE SIGN', 'SUPERSCRIPT TWO', 'MIDDLE DOT', 'EM DASH', 'EN DASH', 'SECTION SIGN',
    'BULLET', 'BLACK CIRCLE', 'BLACK UP-POINTING TRIANGLE', 'WHITE DIAMOND CONTAINING BLACK SMALL DIAMOND',
    'WHITE DIAMOND', 'SINE WAVE', 'CIRCLED PLUS', 'ANTICLOCKWISE OPEN CIRCLE ARROW',
    'DOWNWARDS ARROW', 'BLACK RIGHT-POINTING POINTER', 'SQUARED PLUS',
    'GREATER-THAN OR EQUAL TO', 'LESS-THAN OR EQUAL TO', 'LEFTWARDS ARROW', 'RIGHTWARDS ARROW',
    'PLUS-MINUS SIGN', 'MICRO SIGN', 'OHM SIGN', 'MULTIPLICATION SIGN',
    'LEFT-POINTING DOUBLE ANGLE QUOTATION MARK', 'RIGHT-POINTING DOUBLE ANGLE QUOTATION MARK',
    'BOX DRAWINGS LIGHT VERTICAL AND HORIZONTAL', 'ANTICLOCKWISE OPEN CIRCLE ARROW',
    'RIGHTWARDS HARPOON OVER LEFTWARDS HARPOON', 'LEFTWARDS HARPOON OVER RIGHTWARDS HARPOON'
}

for root, dirs, files in os.walk('ground_station'):
    for f in files:
        if f.endswith('.py'):
            p = os.path.join(root, f)
            with open(p, 'r', encoding='utf-8') as fp:
                for idx, line in enumerate(fp, 1):
                    clean = line.strip()
                    if clean.startswith('#') or clean.startswith('"""'):
                        continue
                    for ch in line:
                        cp = ord(ch)
                        if cp > 127:
                            name = unicodedata.name(ch, '')
                            if 'BOX DRAWINGS' in name or name in allowed_names:
                                continue
                            print(f"{p}:{idx} -> U+{cp:04X} '{ch}' ({name}) in: {clean[:80]}")
