
#I will be using the CSP algorithm.
#find an NxN matrix with numbers from 1 to N showing exactly once in each row and column in a way that the amount of "visible numbers" matches a given input.
#The very first number will determine the size of the matrix
#4 lines will describe the number of visible numbers from
  #top to bottom
  #bottom to top
  #left to right
  #right to left
#number of visibility will determine the number on that index.
#create a matrix with that info

# Constraints:
# All numbers must be unique (no repetition)

#if there is only one visible number then the number must be N

def dfs(board, r, c, N, top, bottom, left, right):
    # If reached end of board -> check all constraints
    if r == N:
        return check_all_constraints(board, N, top, bottom, left, right)

    # Next cell coordinates
    nr, nc = (r, c+1) if c+1 < N else (r+1, 0)

    for val in range(1, N+1):
        board[r][c] = val
        if is_valid(board, r, c, N, top, bottom, left, right):
            if dfs(board, nr, nc, N, top, bottom, left, right):
                return True
        board[r][c] = 0  # backtrack

    return False

def solve_skyscraper_dfs(N, top, bottom, left, right):
    board = [[0] * N for _ in range(N)]
    if dfs(board, 0, 0, N, top, bottom, left, right):
        return board
    return None

def count_visible(seq):
    max_seen, count = 0, 0
    for x in seq:
        if x > max_seen:
            count += 1
            max_seen = x
    return count

def check_all_constraints(board, N, top, bottom, left, right):
    for r in range(N):
        if count_visible(board[r]) != left[r]: return False
        if count_visible(board[r][::-1]) != right[r]: return False
    for c in range(N):
        col = [board[r][c] for r in range(N)]
        if count_visible(col) != top[c]: return False
        if count_visible(col[::-1]) != bottom[c]: return False
    return True

def is_valid(board, r, c, N, top, bottom, left, right):
    # Check row uniqueness
    row = board[r]
    if len(set([x for x in row if x != 0])) != len([x for x in row if x != 0]):
        return False

    # Check column uniqueness
    col = [board[i][c] for i in range(N)]
    if len(set([x for x in col if x != 0])) != len([x for x in col if x != 0]):
        return False

    # If row is complete -> check visibility
    if all(board[r][j] != 0 for j in range(N)):
        if count_visible(board[r]) != left[r]: return False
        if count_visible(board[r][::-1]) != right[r]: return False

    # If column is complete -> check visibility
    col = [board[i][c] for i in range(N)]
    if all(col):
        if count_visible(col) != top[c]: return False
        if count_visible(col[::-1]) != bottom[c]: return False

    return True



if __name__ == "__main__":
    N = int(input().strip())
    top = list(map(int, input().split()))
    bottom = list(map(int, input().split()))
    left = list(map(int, input().split()))
    right = list(map(int, input().split()))

    solution = solve_skyscraper(N, top, bottom, left, right)

    if solution:
        for row in solution:
            print(" ".join(map(str, row)))
    else:
        print("No solution")