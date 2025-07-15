import os
import csv

root_dir = "/Users/antonvalov/Documents/mice_task/data"

for dirpath, _, filenames in os.walk(root_dir):
    for filename in filenames:
        if filename.lower().endswith(".txt"):
            txt_path = os.path.join(dirpath, filename)
            csv_path = os.path.join(dirpath, filename[:-4] + ".csv")

            if os.path.exists(csv_path):
                print(f"Skipping (already exists): {csv_path}")
                continue

            try:
                with open(txt_path, "r", newline="") as infile, open(csv_path, "w", newline="") as outfile:
                    reader = csv.reader(infile, delimiter="\t")
                    writer = csv.writer(outfile)
                    for row in reader:
                        writer.writerow(row)
                print(f"Converted: {txt_path} -> {csv_path}")
            except Exception as e:
                print(f"Error converting {txt_path}: {e}")
