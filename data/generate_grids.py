import json

# Grid-10
link_length10 = {1: 0, 2: 900, 3: 1800, 4: 900, 5: 3600, 6: 900,
                 7: 900, 8: 1800, 9: 900, 10: 0}
edges10 = [(1, 2), (1, 3), (2, 4), (2, 5), (3, 6), (4, 6), (5, 8),
           (6, 8), (3, 7), (4, 7), (7, 9), (8, 10), (9, 10)]

# Save as json file
with open("grid10.json", "w") as f:
    json.dump({"link_length": link_length10, "edges": edges10}, f, indent=2)


# Grid-28
link_length28 = {1: 0, 2: 900, 3: 2700, 4: 1800, 5: 900, 6: 900, 7: 900,
               8: 900, 9: 900, 10: 900, 11: 2700, 12: 900, 13: 900, 14: 900,
               15: 900, 16: 1800, 17: 1800, 18: 900, 19: 2700, 20: 1800, 21: 900,
               22: 900, 23: 900, 24: 900, 25: 1800, 26: 900, 27: 900, 28: 0}

edges28 = [
    (1, 2), (3, 11), (12, 20), # 1st column-row (L shape)
    (5, 13), (14, 22), # 2nd
    (7, 15), (16, 24), # 3rd
    (9, 17), (18, 26), # 4th
    (1, 3), (3, 12), (12, 21), # 1st column
    (5, 14), (14, 23), # 2nd
    (7, 16), (16, 25), # 3rd
    (9, 18), (18, 27), # 4th
    (10, 19), (19, 28), # 5th
    (2, 4), (4, 6), (6, 8), # 1st row
    (11, 13), (13, 15), (15, 17), # 2nd
    (20, 22), (22, 24), (24, 26), # 3rd
    (2, 5), (4, 7), (6, 9), (8, 10), # 1st row-column (L tranposed shape)
    (11, 14), (13, 16), (15, 18), (17, 19), # 2nd
    (20, 23), (22, 25), (24, 27), (26, 28), # 3rd
    ]

# Save as json file
with open("grid28.json", "w") as f:
    json.dump({"link_length": link_length28, "edges": edges28}, f, indent=2)


# Grid-42
link_length42 = {1: 0, 2: 900, 3: 2700, 4: 1800, 5: 900, 6: 900, 7: 900,
               8: 900, 9: 900, 10: 900, 11: 2700, 12: 900, 13: 900, 14: 900,
               15: 900, 16: 1800, 17: 1800, 18: 900, 19: 2700, 20: 1800, 21: 900,
               22: 900, 23: 900, 24: 900, 25: 900, 26: 900, 27: 900, 28: 900,
               29: 900, 30: 900, 31: 900, 32: 900, 33: 2700, 34: 900, 35: 900,
               36: 900, 37: 900, 38: 900, 39: 1800, 40: 900, 41: 900, 42: 0}

edges42 = [
    (1, 2), (3, 11), (12, 20), (21, 29), (30, 38), # 1st column-row (L shape)
    (5, 13), (14, 22), (23, 31), (32, 39), # 2nd
    (7, 15), (16, 24), (25, 33), (34, 40), # 3rd
    (9, 17), (18, 26), (27, 35), (36, 41), # 4th
    (1, 3), (3, 12), (12, 21), (21, 30), # 1st column
    (5, 14), (14, 23), (23, 32), # 2nd
    (7, 16), (16, 25), (25, 34), # 3rd
    (9, 18), (18, 27), (27, 36), # 4th
    (10, 19), (19, 28), (28, 37), (37, 42), # 5th
    (2, 4), (4, 6), (6, 8), # 1st row
    (11, 13), (13, 15), (15, 17), # 2nd
    (20, 22), (22, 24), (24, 26), # 3rd
    (29, 31), (31, 33), (33, 35), # 4th
    (38, 39), (39, 40), (40, 41), # 5th
    (2, 5), (4, 7), (6, 9), (8, 10), # 1st row-column (L tranposed shape)
    (11, 14), (13, 16), (15, 18), (17, 19), # 2nd
    (20, 23), (22, 25), (24, 27), (26, 28), # 3rd
    (29, 32), (31, 34), (33, 36), (35, 37), # 4rd
    (41, 42) # 5th
    ]

# Save as json file
with open("grid42.json", "w") as f:
    json.dump({"link_length": link_length42, "edges": edges42}, f, indent=2)


# Grid-51
link_length51 = {1: 0, 2: 900, 3: 2700, 4: 1800, 5: 900, 6: 900, 7: 900,
               8: 900, 9: 900, 10: 900, 11: 2700, 12: 900, 13: 900, 14: 900,
               15: 900, 16: 1800, 17: 1800, 18: 900, 19: 2700, 20: 1800, 21: 900,
               22: 900, 23: 900, 24: 900, 25: 900, 26: 900, 27: 900, 28: 900,
               29: 900, 30: 900, 31: 900, 32: 900, 33: 2700, 34: 900, 35: 900,
               36: 900, 37: 900, 38: 900, 39: 1800, 40: 900, 41: 900, 42: 900,
               43: 900, 44: 2700, 45: 900, 46: 900, 47: 900, 48: 900, 49: 1800,
               50: 900, 51: 0}

edges51 = [
    (1, 2), (3, 11), (12, 20), (21, 29), (30, 38), (43, 44), # 1st column-row (L shape)
    (5, 13), (14, 22), (23, 31), (32, 39), (45, 46), # 2nd
    (7, 15), (16, 24), (25, 33), (34, 40), (47, 48), # 3rd
    (9, 17), (18, 26), (27, 35), (36, 41), (49, 50), # 4th
    (1, 3), (3, 12), (12, 21), (21, 30), (30, 43), # 1st column
    (5, 14), (14, 23), (23, 32), (32, 45), # 2nd
    (7, 16), (16, 25), (25, 34), (34, 47), # 3rd
    (9, 18), (18, 27), (27, 36), (36, 49), # 4th
    (10, 19), (19, 28), (28, 37), (37, 42), (42, 51), # 5th
    (2, 4), (4, 6), (6, 8), # 1st row
    (11, 13), (13, 15), (15, 17), # 2nd
    (20, 22), (22, 24), (24, 26), # 3rd
    (29, 31), (31, 33), (33, 35), # 4th
    (38, 39), (39, 40), (40, 41), # 5th
    (44, 46), (46, 48), (48, 50), # 6th
    (2, 5), (4, 7), (6, 9), (8, 10), # 1st row-column (L tranposed shape)
    (11, 14), (13, 16), (15, 18), (17, 19), # 2nd
    (20, 23), (22, 25), (24, 27), (26, 28), # 3rd
    (29, 32), (31, 34), (33, 36), (35, 37), # 4rd
    (38, 45), (39, 47), (40, 49), (41, 42), # 5th
    (50, 51) # 6th
    ]

# Save as json file
with open("grid51.json", "w") as f:
    json.dump({"link_length": link_length51, "edges": edges51}, f, indent=2)


# Grid-82
# The length of the link is 900 for every link except source and sink that have 0
link_length82 = {i: 0 if i in (1, 82) else 900 for i in range(1, 83)}

edges82 = [
    (1, 2), (3, 21), (22, 38), (39, 56), (57, 73), # 1st column-row (L shape)
    (5, 23), (24, 40), (41, 58), (59, 74), # 2nd
    (7, 25), (60, 75), # 3rd
    (9, 26), (27, 44), (45, 61), (62, 76), # 4th
    (11, 28), (29, 46), (63, 77), # 5th
    (30, 48), (49, 64), (65, 78), # 6th
    (15, 31), (32, 50), (51, 66), (67, 79), # 7th
    (17, 33), (34, 52), (69, 80), # 8th
    (19, 35), (36, 53), (54, 70), (71, 81), # 9th
    (1, 3), (3, 22), (22, 39), (39, 57), # 1st column
    (5, 24), (24, 41), (41, 59), # 2nd
    (43, 60), # 3rd
    (9, 27), (27, 45), (45, 62), # 4th
    (11, 29), (29, 47), (47, 63), # 5th
    (13, 30), (30, 49), (49, 65), # 6th
    (15, 32), (32, 51), (51, 67), # 7th
    (17, 34), # 8th
    (19, 36), (36, 54), (54, 71), # 9th
    (20, 37), (37, 55), (55, 72), (72, 82), # 10th
    (2, 4), (4, 6), (6, 8), (8, 10), (10, 12), (12, 14), (14, 16), (16, 18), # 1st row
    (21, 23), (23, 25), (25, 26), (26, 28), (31, 33), (33, 35), # 2nd
    (38, 40), (40, 42), (42, 44), (44, 46), (46, 48), (48, 50), (50, 52), (52, 53), # 3rd
    (56, 58), (64, 66), (66, 68), (68, 70), # 4th
    (73, 74), (74, 75), (75, 76), (76, 77), (77, 78), (78, 79), (79, 80), (80, 81), # 5th
    (2, 5), (4, 7), (6, 9), (8, 11), (10, 13), (12, 15), (14, 17), (16, 19), (18, 20), # 1st row-column (L tranposed shape)
    (21, 24), (25, 27), (26, 29), (28, 30), (31, 34), (33, 36), (35, 37), # 2nd
    (38, 41), (40, 43), (42, 45), (44, 47), (46, 49), (48, 51), (52, 54), (53, 55), # 3rd
    (56, 59), (58, 60), (61, 63), (64, 67), (66, 69), (68, 71), (70, 72), # 4th
    (81, 82) # 5th
    ]

with open("grid82.json", "w") as f:
    json.dump({"link_length": link_length82, "edges": edges82}, f, indent=2)