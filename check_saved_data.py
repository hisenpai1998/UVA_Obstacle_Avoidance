# -------------------------------------------------------------------------
# Copyright (c) 2025 Hieu Tran Quang and Duc Huy Vu
# All rights reserved.
#
# This source code is part of a private academic project submitted for
# educational purposes only. It may be viewed and assessed by authorized
# instructors and examiners as part of academic evaluation.
#
# Unauthorized use, reproduction, distribution, or modification of this
# code, in whole or in part, is strictly prohibited without the prior
# written consent of the authors.
#
# This file and project are NOT open source.
# -------------------------------------------------------------------------

import pickle
from collections import Counter

labels = []
label_strs = []

# Load labels and label_strs
with open('1.pkl', 'rb') as f:
    try:
        while True:
            frame = pickle.load(f)
            labels.append(frame.get("label"))
            label_strs.append(frame.get("label_str"))
    except EOFError:
        pass

# Count pairs of (label, label_str)
combined = list(zip(labels, label_strs))
counted = Counter(combined)

# Sort by label
sorted_items = sorted(counted.items(), key=lambda x: x[0][0])  # x[0][0] is the numeric label

# Print header
print("Label | Name            | Count")
print("------|-----------------|------")

total = 0
# Print each row
for (label, name), count in sorted_items:
    print(f"{label:<5} | {name:<15} | {count}")
    total += count

# Print total
print("------|-----------------|------")
print(f"Total |                 | {total}")
