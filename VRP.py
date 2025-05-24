import sys
from ortools.linear_solver import pywraplp
f=sys.stdin
[N,M,K]=[int(x) for x in f.readline().split()]
c=[]
for i in range(N) :
    row=[int(x) for x in f.readline().split()]
    c.append(row)
solver=pywraplp.Solver.CreateSolver('SCIP')

x={}
for i in range(1,N+1) :
    for j in range(1,N+1) :
        if i!=j :
            for k in range(1,K+1) :
                x[i,j,k]=solver.BoolVar(f'x_{i}_{j}_{k}')
# Each city can only be visited by once vehicle  once time
for i in range(2,N+1) :
    solver.Add(solver.Sum(x[i,j,k] for j in range(1,N+1) for k in range(1,K+1) if i!=j)==1)
for k in range(1,K+1) :
    solver.Add(solver.Sum(x[1,j,k] for j in range(2,N+1))==1)
    solver.Add(solver.Sum(x[i,1,k] for i in range(2,N+1))==1)
# Flow balance 
for k in range(1, K + 1):
    for h in range(1, N + 1):
        if h == 1:
            continue
        solver.Add(solver.Sum(x[i, h, k] for i in range(1, N + 1) if i != h) ==
                   solver.Sum(x[h, j, k] for j in range(1, N + 1) if j != h))
# 1 direction travel :
# Subtour elimination :
u = {}
for k in range(1, K+1):
    for i in range(1, N+1):  # bao gồm cả i = 1
        # Tại điểm khởi đầu (i=1), đặt u = 1
        if i == 1:
            u[i, k] = solver.IntVar(1, 1, f'u_{i}_{k}')
        else:
            u[i, k] = solver.IntVar(2, N , f'u_{i}_{k}')


for k in range(1,K+1) :
    for i in range(2,N+1) :
        for j in range(2,N+1) :
            if i!=j :
                solver.Add(u[i, k] - u[j, k] + (N - 1) * x[i, j, k] <= N - 2)

solver.Minimize(sum(x[i,j,k]*c[i-1][j-1] for i in range(1,N+1) for j in range(1,N+1) for k in range(1,K+1) if i!=j))
status=solver.Solve()
if status==pywraplp.Solver.OPTIMAL :
    print(solver.Objective().Value())
    for k in range(1, K + 1):
        route = [1]
        current = 1
        while True:
            found = False
            for j in range(1, N + 1):
                if j != current and x[current, j, k].solution_value() > 0.5:
                    route.append(j)
                    current = j
                    found = True
                    break
            if not found or current == 1:
                break
        if len(route) > 1:
            route.append(1)
            print(f'Route for vehicle {k}:', ' -> '.join(map(str, route)))
else :
    print(0)




