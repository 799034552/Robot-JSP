import os
import time
import numpy as np
import heapq

# np.linspace(st,ed,num) (下界，上界，步数)
grid = [
    (1, 2, 2),
    (3, 4, 3),
    (5, 6, 4),
        ]
maps = [5, 6, 7, 8]
topk = 3
ans = []

def grid_search(cur_val:list, depth:int):

    if depth == len(grid):
        scores = 0
        for map_idx in maps:
            cmd = "Robot.exe -m ./maps/" + str(map_idx) + ".txt \"build/main.exe" 
            for val in cur_val:
                cmd += " " + str(val)
            cmd += "\" -f -l WARN"
            f = os.popen(cmd)
            out = f.read().strip()
            f.close()
            score = int(eval(out)['score'])
            scores += score

        if len(ans) == topk:
            heapq.heappushpop(ans, (scores, cur_val.copy()))
        else:
            heapq.heappush(ans, (scores, cur_val.copy()))
        return

    vals = np.linspace(*grid[depth])
    for val in vals:
        cur_val.append(val)
        grid_search(cur_val, depth+1)
        cur_val.pop()
    return

def run_one():
    scores = 0
    for map_idx in maps:
        cmd = "Robot.exe -m ./maps/" + str(map_idx) + ".txt \"build/main.exe\" -f -l WARN"
        f = os.popen(cmd)
        out = f.read().strip()
        print(out)
        f.close()
        score = int(eval(out)['score'])
        scores += score
    print("default total:", scores)
    return

def main():
    t0 = time.time()
    run_one()
    # exit()
    t = time.time() - t0
    print("one cost:", t)

    num = 1
    for _ in grid:
        num *= _[2]
    print("search nums:", num, "total cost:", num*t)

    # t = time.time()
    grid_search([], 0)
    ans.sort(reverse=True)
    for res in ans:
        print(res)
    # print("total cost:", time.time()-t)
    return

if __name__ == "__main__":
    main()

# int main(int argc, char* argv[]) {
#     // 用于调参
#     if (argc > 1) {
#         double val = strtod(argv[1], nullptr);
#         // XXX = val;
#     }
#     else { //默认值
#         // XXX = DEFAULT_VAL;
#     }
