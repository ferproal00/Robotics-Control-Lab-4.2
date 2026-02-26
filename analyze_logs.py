import re
import numpy as np
import csv

input_file = "raw_log.txt"
output_csv = "parsed_data.csv"
summary_file = "metrics_summary.txt"

pattern = re.compile(
    r"pos=\[(.*?)\].*?des=\[(.*?)\].*?err=\[(.*?)\].*?v=\[(.*?)\]"
)

data = []

with open(input_file, "r") as f:
    for line in f:
        match = pattern.search(line)
        if match:
            pos = list(map(float, match.group(1).split()))
            des = list(map(float, match.group(2).split()))
            err = list(map(float, match.group(3).split()))
            v   = list(map(float, match.group(4).split()))

            v_norm = np.linalg.norm(v)

            data.append(pos + des + err + v + [v_norm])

data = np.array(data)

# Column indices
err_cols = data[:, 6:9]
v_cols   = data[:, 9:12]
v_norm   = data[:, 12]

# Metrics
rmse_xyz = np.sqrt(np.mean(err_cols**2, axis=0))
rmse_total = np.sqrt(np.mean(np.sum(err_cols**2, axis=1)))
max_err_xyz = np.max(np.abs(err_cols), axis=0)
max_err_total = np.max(np.linalg.norm(err_cols, axis=1))

v_mean = np.mean(v_norm)
v_max = np.max(v_norm)

# Save CSV
header = [
    "pos_x","pos_y","pos_z",
    "des_x","des_y","des_z",
    "err_x","err_y","err_z",
    "vx","vy","vz",
    "v_norm"
]

with open(output_csv, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(header)
    writer.writerows(data)

# Save summary
with open(summary_file, "w") as f:
    f.write("===== METRICS SUMMARY =====\n")
    f.write(f"Samples: {len(data)}\n\n")

    f.write("RMSE per axis (x,y,z):\n")
    f.write(f"{rmse_xyz}\n\n")

    f.write(f"RMSE total: {rmse_total}\n\n")

    f.write("Max absolute error per axis:\n")
    f.write(f"{max_err_xyz}\n\n")

    f.write(f"Max absolute error total: {max_err_total}\n\n")

    f.write(f"Velocity mean norm: {v_mean}\n")
    f.write(f"Velocity max norm: {v_max}\n")

print("CSV generado:", output_csv)
print("Resumen generado:", summary_file)