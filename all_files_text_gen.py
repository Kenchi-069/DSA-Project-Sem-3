import os

folders = ["Phase-1", "Phase-2", "Phase-3"]

with open("all_files.txt", "w") as out:
    for folder in folders:
        for fname in os.listdir(folder):
            if fname.endswith((".cpp", ".hpp")):
                path = os.path.join(folder, fname)
                out.write(f"=== {path} ===\n")
                with open(path, "r") as f:
                    out.write(f.read())
                out.write("\n\n")
