import os
import numpy as np

st = 6.0
ed = 8.0
num = 5
st1 = 6.0
ed1 = 8.0
num1 = 5
st2 = 6.0
ed2 = 8.0
num2 = 5
topk = 3
maps = [1, 2, 3, 4]

vals = np.linspace(st, ed, num)
vals1 = np.linspace(st1, ed1, num1)

vals2 = np.linspace(st2, ed2, num2)

# print("maps:", maps, "vals:", vals)

def run_one():
    os.system("Robot.exe \"build/main.exe\" -f -l WARN -m ./maps/1.txt")
    os.system("Robot.exe \"build/main.exe\" -f -l WARN -m ./maps/2.txt")
    os.system("Robot.exe \"build/main.exe\" -f -l WARN -m ./maps/3.txt")
    os.system("Robot.exe \"build/main.exe\" -f -l WARN -m ./maps/4.txt")

def best_k():
    ans = []
    for val in vals:
        scores = 0
        for map_idx in maps:
            cmd = "Robot.exe \"build/main.exe " + str(val) + "\" -f -l WARN -m ./maps/" + str(map_idx) + ".txt"
            f = os.popen(cmd)
            out = f.read().strip()

            score = int(eval(out)['score'])
            scores += score
        ans.append(scores)

    res = np.argsort(ans)

    for i in range(topk):
        idx = res[-(i+1)]
        print("val:", vals[idx], "| all-scores:", ans[idx])

best_k()
# run_one()


# int main(int argc, char* argv[]) {
#     // 用于调参
#     if (argc == 2) {
#         double val = strtod(argv[1], nullptr);
#         // XXX = val;
#     }
#     else { //默认值
#         // XXX = DEFAULT_VAL;
#     }
