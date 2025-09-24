# 3x3 보드: 0=empty, 1=O, 2=X
int8[9] cells  

# 최근 갱신된 셀 인덱스(-1이면 없음)
int8 last_move

# 보드가 유효하게 읽혔는지
bool valid

# "playing"/"win_O"/"win_X"/"draw"
string status
