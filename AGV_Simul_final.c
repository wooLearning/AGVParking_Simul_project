//// =============================================================================
//// ���� ������Ʈ ������ �ùķ��̼� �ý���
//// =============================================================================
//// �˰���: D* Lite + WHCA* + WFG(SCC) + Partial CBS
//// �ֿ� ���:
////   - �� ���� (1~5): Map#1 = �⺻, Map#2~5 = ��Ʈ���� �׽�Ʈ
////   - ��ȭ�� ����: �Ͻ�����, ���� ����, �ӵ� ����, ����
////   - ������ ���� �ǽð� ������
//// =============================================================================
//
//#define _CRT_SECURE_NO_WARNINGS
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <math.h>
//#include <float.h>
//#include <time.h>
//#include <stdarg.h>
//#include <ctype.h>
//
//#include <windows.h>
//
//#include <psapi.h>
//#include <conio.h>
//#ifdef _WIN32
//#pragma comment(lib, "psapi.lib")
//#endif
///**
// * @brief Windows ȯ�濡�� �и��� ���� ��⸦ �����ϴ� ������ ���� ��ũ���Դϴ�.
// * @param ms ����� �и���(ms)
// */
//#define sleep_ms(ms) Sleep(ms)
//
// /**
//  * @brief ���� ���ڿ��� ���ۿ� �����ϰ� �̾���̴� ���� ��ũ���Դϴ�.
//  *        snprintf ��ȯ���� ����Ͽ� ������/���� �뷮�� �ڵ� �����մϴ�.
//  * @param P    ��� ������(char*)
//  * @param REM  ���� �뷮(size_t)
//  * @param ...  printf ��Ÿ�� ���� ���� ����
//  */
//#ifndef APPEND_FMT
//#define APPEND_FMT(P, REM, /*fmt, ...*/ ...)                            \
//    do {                                                                \
//        int __w = snprintf((P), (REM), __VA_ARGS__);                    \
//        if (__w < 0) __w = 0;                                           \
//        if ((size_t)__w > (REM)) __w = (int)(REM);                      \
//        (P) += __w;                                                     \
//        (REM) -= __w;                                                   \
//    } while (0)
//#endif
//
//
//  // =============================================================================
//  // ���� 1: ��� �� ����
//  // =============================================================================
//  /**
//   * @brief ���� ���/��ũ�� �� UI���˰��� �⺻ ������ ���� �����Դϴ�.
//   */
//   /**
//    * @brief ���� ��� �� �⺻ ���� ũ�� ����
//    * - TRUE/FALSE: ������ �Ҹ��� ���
//    * - INPUT_BUFFER_SIZE: �ܼ� �Է� ���� ũ��
//    * - DISPLAY_BUFFER_SIZE: �ؽ�Ʈ UI ������ ���� ũ��
//    */
//#define TRUE 1
//#define FALSE 0
//#define INPUT_BUFFER_SIZE 500
//#define DISPLAY_BUFFER_SIZE 512000
//
//    /**
//     * @brief ANSI ���� �ڵ� ��� (���� �͹̳� Ȱ��ȭ �� �÷� ��¿� ���)
//     */
//     // --- ANSI ���� �ڵ� ---
//#define C_NRM "\x1b[0m"
//#define C_RED "\x1b[31m"
//#define C_GRN "\x1b[32m"
//#define C_YEL "\x1b[33m"
//#define C_BLU "\x1b[34m"
//#define C_MAG "\x1b[35m"
//#define C_CYN "\x1b[36m"
//#define C_WHT "\x1b[37m"
//#define C_GRY "\x1b[90m"
//#define C_B_RED "\x1b[1;31m"
//#define C_B_GRN "\x1b[1;32m"
//#define C_B_YEL "\x1b[1;33m"
//#define C_B_MAG "\x1b[1;35m"
//#define C_B_CYN "\x1b[1;36m"
//#define C_B_WHT "\x1b[1;37m"
//
///**
// * @brief ������Ʈ ���� �ȷ�Ʈ(A..J) ����. �� �ε����� ������Ʈ ID�� �����մϴ�.
// */
// // --- ������Ʈ ���� �ȷ�Ʈ (A..J) ---
//static const char* AGENT_COLORS[10] = {
//C_B_CYN, C_B_YEL, C_B_MAG, C_B_GRN, C_B_RED,
//C_B_WHT, C_CYN,   C_YEL,   C_MAG,   C_GRN
//};
//
//
///**
// * @brief �׸���/������Ʈ ���� ����
// * - GRID_WIDTH/HEIGHT: �� ũ��
// * - MAX_AGENTS: ���� ������Ʈ ��
// * - MAX_GOALS: �ִ� ����ĭ ����(�׸��� ���� ����)
// * - INF: ��� ��� ���Ѵ� ǥ��
// */
// // --- �׸��� �� ������Ʈ ���� ---
//#define GRID_WIDTH  82
//#define GRID_HEIGHT 42
//#define MAX_AGENTS  10
//#define MAX_GOALS   (GRID_WIDTH*GRID_HEIGHT)
//#define INF 1e18
//#define NUM_DIRECTIONS 4
//#ifndef DIR4_COUNT
//#define DIR4_COUNT 4
//#endif
//
///**
// * @brief 4����(�������) �� ���� ���¸� ��Ÿ���� ���� ������
// */
//typedef enum {
//    DIR_NONE = -1,
//    DIR_UP = 0,
//    DIR_RIGHT = 1,
//    DIR_DOWN = 2,
//    DIR_LEFT = 3
//} AgentDir;
//
///**
// * @brief �ùķ��̼� ���� �Ű�����
// * - DISTANCE_BEFORE_CHARGE: ���� �Ǵ��� ���� ���� �̵� �Ÿ� ����
// * - CHARGE_TIME: ���� �ҿ� ƽ
// * - MAX_CHARGE_STATIONS: �ִ� ������ ��
// * - MAX_PHASES: ����� ���� �ó����� �ܰ� �� ����
// * - REALTIME_MODE_TIMELIMIT: �ǽð� ��� �� ƽ ����
// * - DASHBOARD_INTERVAL_STEPS: �ǽð� ��ú��� ��� ����
// * - MAX_TASKS: �ǽð� ��⿭ �ִ� �۾� ��
// * - MAX_SPEED_MULTIPLIER: �ִ� ���
// */
// // --- �ùķ��̼� �Ű����� ---
//#define DISTANCE_BEFORE_CHARGE 300.0
//#define CHARGE_TIME 20
//#define MAX_CHARGE_STATIONS 10
//#define MAX_PHASES 20
//#define REALTIME_MODE_TIMELIMIT 1000000
//#define DASHBOARD_INTERVAL_STEPS 2500
//#define MAX_TASKS 50
//#define MAX_SPEED_MULTIPLIER 10000.0f
//#define EVENT_GENERATION_INTERVAL 10
//// ���� �ܰ迡�� ���� ����ȭ�� �����ϱ� �� ����� ���� ��
//#ifndef CLEANUP_FORCE_IDLE_AFTER_STEPS
//#define CLEANUP_FORCE_IDLE_AFTER_STEPS 11
//#endif
//
///**
// * @brief UI ���� ���� ���
// * - LOG_BUFFER_LINES: ��ȯ �α� �� ��
// * - LOG_BUFFER_WIDTH: �α� �� �� �ʺ�
// * - STATUS_STRING_WIDTH: ���� �г� ���� ��
// */
// // --- UI ���� ---
//#define LOG_BUFFER_LINES 5
//#define LOG_BUFFER_WIDTH 256
//#define STATUS_STRING_WIDTH 25
//
///**
// * @brief UI Ÿ�̹� �� ������ ���� ����
// * - PAUSE_POLL_INTERVAL_MS: �Ͻ����� ���� ���� �ֱ�
// * - RENDER_STRIDE_MAX/MIN: ������ ��ŵ ����
// */
// // --- UI Ÿ�̹� �� ������ ���� ---
//#ifndef PAUSE_POLL_INTERVAL_MS
//#define PAUSE_POLL_INTERVAL_MS 50
//#endif
//#ifndef RENDER_STRIDE_MAX
//#define RENDER_STRIDE_MAX 8
//#endif
//#ifndef RENDER_STRIDE_MIN
//#define RENDER_STRIDE_MIN 1
//#endif
//
///**
// * @brief �۾�/�켱����/���� ����ġ ����
// * - TASK_ACTION_TICKS: ����/���� �۾� ��� ƽ
// * - PRIORITY_*: ���º� �켱���� �⺻��
// * - STUCK_BOOST_*: stuck ������ ���� ����
// */
// // --- �۾� �� �켱���� ��� ---
//#ifndef TASK_ACTION_TICKS
//#define TASK_ACTION_TICKS 10         // ����/���� �۾� ��� ƽ ��
//#endif
//#ifndef PRIORITY_RETURNING_WITH_CAR
//#define PRIORITY_RETURNING_WITH_CAR 3
//#endif
//#ifndef PRIORITY_GOING_TO_CHARGE
//#define PRIORITY_GOING_TO_CHARGE 2
//#endif
//#ifndef PRIORITY_MOVING_TASK
//#define PRIORITY_MOVING_TASK 1       // GOING_TO_PARK / GOING_TO_COLLECT ���� �켱����
//#endif
//#ifndef STUCK_BOOST_MULT
//#define STUCK_BOOST_MULT 10
//#endif
//#ifndef STUCK_BOOST_HARD
//#define STUCK_BOOST_HARD 1000        // stuck_steps >= DEADLOCK_THRESHOLD�� �� ����
//#endif
//
///**
// * @brief ���� ���� �Ǵ� �Ӱ�ġ ����(stuck ���� ƽ ��)
// */
// // --- ���� ���� ó�� ---
//#define DEADLOCK_THRESHOLD 5
//
///**
// * @brief WHCA* ȣ������ ����(�ּ�/�ִ�/�ʱⰪ)
// */
// // --- WHCA* ȣ������ ���� ---
//#define MIN_WHCA_HORIZON 5
//#define MAX_WHCA_HORIZON 11
//static int g_whca_horizon = 5;
//
///**
// * @brief ��� �׷���(WFG) �� Partial CBS ���� �ѵ� ����
// */
// // --- WFG �� CBS �Ű����� 
//#define MAX_WAIT_EDGES 64
//#define MAX_CBS_GROUP  5
//#define MAX_CBS_CONS   64
//#define MAX_CBS_NODES  128
//#define CBS_MAX_EXPANSIONS 64
//
///**
// * @brief ST-A*�� ���� ���� �ε��� ���� (�޸� ���Ҵ� ũ��)
// */
// // --- ST-A* ���� ���� ũ�� ---
//#define MAX_TOT (((MAX_WHCA_HORIZON)+1) * GRID_WIDTH * GRID_HEIGHT)
//
///**
// * @brief �ؽ�Ʈ UI(����/��/�α�)�� �� ���� ��� ����ϱ� ���� ���� ����
// */
//static char g_display_buf[DISPLAY_BUFFER_SIZE];
//
//
//// =============================================================================
//// ���� 2: ������ ����
//// =============================================================================
//// D* Lite �켱���� ť���� ����ϴ� Ű �� ����ü
//typedef struct { double k1; double k2; } Key;
//// Key ����ü ���� ���� �Լ�
//static inline Key make_key(double a, double b) { Key k; k.k1 = a; k.k2 = b; return k; }
//
//// �׸��� ���� �� ĭ(���)�� ��Ÿ���� ����ü
//typedef struct Node {
//    int x, y;               // ����� �׸��� ��ǥ
//    int is_obstacle;        // ��ֹ� ���� (����)
//    int is_goal;            // ���� ���� ����
//    int is_temp;            // �ӽ� ��ֹ� ���� (��ȹ��)
//    int is_parked;          // ������ �����Ǿ� �ִ��� ����
//    int reserved_by_agent;  // �ٸ� ������Ʈ�� ���� ����Ǿ����� ���� (-1: ���� ����)
//} Node;
//
//// --- ������Ʈ ���� ������ ---
//typedef enum {
//    IDLE, GOING_TO_PARK, RETURNING_HOME_EMPTY, GOING_TO_COLLECT,
//    RETURNING_WITH_CAR, GOING_TO_CHARGE, CHARGING, RETURNING_HOME_MAINTENANCE
//} AgentState;
//
//// --- �ó����� Ÿ�� ---
//typedef enum { PARK_PHASE, EXIT_PHASE } PhaseType;
//typedef struct { PhaseType type; int task_count; char type_name[10]; } DynamicPhase;
//typedef enum { TASK_NONE, TASK_PARK, TASK_EXIT } TaskType;
//typedef struct TaskNode { TaskType type; int created_at_step; struct TaskNode* next; } TaskNode;
//typedef enum { MODE_UNINITIALIZED, MODE_CUSTOM, MODE_REALTIME } SimulationMode;
//
//// --- ��� ��ȹ �˰��� ���� ---
//typedef enum {
//    PATHALGO_DEFAULT = 0,       // ����: WHCA* + D* Lite + WFG(SCC) + Partial CBS
//    PATHALGO_ASTAR_SIMPLE = 1,  // �ܼ� A* �� ���� ��ȹ
//    PATHALGO_DSTAR_BASIC = 2    // �⺻ ����/�浹 ó���� ���� ������ D* Lite
//} PathAlgo;
//
//// --- �׸��� �� ����ü ---
//typedef struct {
//    Node grid[GRID_HEIGHT][GRID_WIDTH];
//    Node* goals[MAX_GOALS];
//    int num_goals;
//    Node* charge_stations[MAX_CHARGE_STATIONS];
//    int num_charge_stations;
//} GridMap;
//
//// --- ������ D* Lite Ž�� �� (������Ʈ�� ������) ---
//typedef struct {
//    double g, rhs;      // D* Lite�� g���� rhs��
//    Key key;            // �켱���� ť ���Ŀ� ���Ǵ� Ű
//    int in_pq;          // �켱���� ť(open list) ���� ����
//    int pq_index;       // �켱���� ť ���� �ε��� (�� �����)
//} SearchCell;
//
//// --- D* Lite�� �켱���� ť (�� ����) ---
//typedef struct {
//    Node** nodes;       // ��� ������ �迭 (��)
//    int size;           // ���� ť�� ����� ����� ��
//    int capacity;       // ť�� �ִ� �뷮
//} NodePQ;
//
//// Pathfinder ����ü ���� ����
//struct Pathfinder_;
//struct Agent_;  // Agent ���� ����
//typedef struct Agent_ Agent;  // Agent Ÿ�� ��Ī
//// D* Lite �˰����� �ν��Ͻ��� �����ϴ� ����ü (������Ʈ���� ����)
//typedef struct Pathfinder_ {
//    NodePQ pq;                                  // �켱���� ť (open list)
//    SearchCell cells[GRID_HEIGHT][GRID_WIDTH];  // �� ���� D* Lite ������
//    Node* start_node;                           // ���� ������Ʈ�� ���� ��ġ (s_start)
//    Node* goal_node;                            // ���� ��ǥ ��ġ (t, rhs=0)
//    Node* last_start;                           // ���� ������ ���� ��ġ (km ����)
//    double km;                                  // D* Lite�� Ű ������ (key modifier)
//    const struct Agent_* agent;                 // �� Pathfinder�� ������ ������Ʈ (��� üũ��)
//} Pathfinder;
//
//// --- WHCA* �ð� Ȯ�� ���� ���̺� ---
//typedef struct {
//    int occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH];
//} ReservationTable;
//
//// --- ��� �׷��� (�浹 ������ ������ �ð� Ȯ����) ---
//typedef enum { CAUSE_VERTEX = 0, CAUSE_SWAP = 1 } CauseType;
//typedef struct {
//    int from_id, to_id;   // ��� ���� (from_id�� to_id�� ��ٸ�)
//    int t;                // �浹�� �߻��� �ð� (1..H)
//    CauseType cause;      // �浹 ���� (���� �Ǵ� ����)
//    int x1, y1;           // �浹 ��ġ 1 (������ ��� ���⸸ ���)
//    int x2, y2;           // �浹 ��ġ 2 (������ ��� ���)
//} WaitEdge;
//
//// --- �κ� �� CBS (Conflict-Based Search) ���� ���� ---
//typedef struct {
//    int agent;      // ���� ������ ����� ������Ʈ ID
//    int t;          // ���� ���� �ð�
//    int is_edge;    // 0: ���� ����, 1: ���� ����
//    int x, y;       // ���� ��ǥ �Ǵ� ������ ���� ��ǥ
//    int tox, toy;   // ������ ���� ��ǥ
//} CBSConstraint;
//
//// CBS ����� Ž�� ���
//typedef struct {
//    CBSConstraint cons[MAX_CBS_CONS];                   // ���� ��忡 ����� ���� ���� ���
//    int ncons;                                          // ���� ������ ��
//    Node* plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1];      // ���� ������ �����ϴ� �� ������Ʈ�� ��� ��ȹ
//    double cost;                                        // ���� ��ȹ�� �� ���
//} CBSNode;
//
//
//// --- ȸ�� ���� �Լ� (Agent ����ü���� ���� ��ġ�Ͽ� ���� �����ϵ��� ��) ---
//// ��Ÿ �����κ��� ���� ���
//static inline AgentDir dir_from_delta(int dx, int dy) {
//    if (dx == 1 && dy == 0) return DIR_RIGHT;
//    if (dx == -1 && dy == 0) return DIR_LEFT;
//    if (dx == 0 && dy == -1) return DIR_UP;
//    if (dx == 0 && dy == 1) return DIR_DOWN;
//    return DIR_NONE;
//}
//
//// �� ���� �� ȸ���� �ʿ��� ���� �� ���
//static inline int dir_turn_steps(AgentDir from, AgentDir to) {
//    if (from == DIR_NONE || to == DIR_NONE) return 0;
//    int diff = ((int)to - (int)from + NUM_DIRECTIONS) % NUM_DIRECTIONS;
//    return diff <= 2 ? diff : 4 - diff;;
//}
//
//// --- �̵� �� ���� ���� (����) ---
//// ȸ�� ��� �ð� (90�� ȸ�� �� �� ��� ƽ)
//#ifndef TURN_90_WAIT
//#define TURN_90_WAIT 2
//#endif
//
//// 4���� �⺻ ������ (��� ����)
//// �ε���: 0=UP, 1=DOWN, 2=RIGHT, 3=LEFT
//static const int DIR4_X[4] = { 0, 0, 1, -1 };
//static const int DIR4_Y[4] = { 1, -1, 0, 0 };
//// 5���� ������ (���� ����)
//// �ε��� 0 = STAY, ���� RIGHT/LEFT/DOWN/UP
//static const int DIR5_X[5] = { 0, 1, -1, 0, 0 };
//static const int DIR5_Y[5] = { 0, 0,  0, 1,-1 };
//#ifndef DIR5_COUNT
//#define DIR5_COUNT 5
//#endif
//
//// --- ����ư �Ÿ� ���� (�ߺ� ���� ����) ---
//// ��� �� ����ư �Ÿ� ���
//static inline double manhattan_nodes(const Node* a, const Node* b) {
//    return fabs((double)a->x - (double)b->x) + fabs((double)a->y - (double)b->y);
//}
//// ��ǥ �� ����ư �Ÿ� ���
//static inline double manhattan_xy(int x1, int y1, int x2, int y2) {
//    return fabs((double)x1 - (double)x2) + fabs((double)y1 - (double)y2);
//}
//
//// --- �ӽ� ��ֹ� ��ŷ (��ȹ �ܰ� ����) ---
//// - ��ȹ �� �浹 ȸ�Ǹ� ���� �������� ��ֹ��� �����ϰ� �����ϴ� ���
//// - ��� �Ŀ��� �ݵ�� temp_unmark_all �Ǵ� ���� ���ؽ�Ʈ ���� �Լ��� ȣ���ؾ� ��
//#ifndef TEMP_MARK_MAX
//#define TEMP_MARK_MAX 128
//#endif
//typedef struct {
//    Node* nodes[TEMP_MARK_MAX];
//    int count;
//} TempMarkList;
//// �ӽ� ��ŷ ����Ʈ �ʱ�ȭ
//static inline void temp_mark_init(TempMarkList* l) { if (l) l->count = 0; }
//// ��带 �ӽ� ��ֹ��� ��ŷ
//static inline void temp_mark_node(TempMarkList* l, Node* n) {
//    if (!l || !n) return;
//    if (!n->is_temp) {
//        n->is_temp = TRUE;
//        if (l->count < TEMP_MARK_MAX) l->nodes[l->count++] = n;
//    }
//}
//// ��� �ӽ� ��ŷ ����
//static inline void temp_unmark_all(TempMarkList* l) {
//    if (!l) return;
//    for (int i = 0; i < l->count; i++) if (l->nodes[i]) l->nodes[i]->is_temp = FALSE;
//    l->count = 0;
//}
//
//// --- �ӽ� ��ŷ ���� + D* Lite ȯ�� ��ȭ ���� ---
//// ����: ���� Ž�� �˰����� �ϰ����� �����Ϸ���, �ӽ� ��ֹ� ���� ��
//// �ش� ���� ���� ��ȭ�� D* Lite�� ����(notify)�ؾ� �մϴ�.
//// �� ����� ���� ������Ÿ�� ���� ���Ŀ� �Լ� ������ ���ǵ˴ϴ�.
//
//// ������Ʈ�� ��� ���¿� �Ӽ��� �����ϴ� ����ü
//typedef struct Agent_ {
//    int id;                     // ������Ʈ ���� ID
//    char symbol;                // ȭ�鿡 ǥ�õ� ���� (A, B, ...)
//    Node* pos;                  // ���� ��ġ
//    Node* home_base;            // ���� ��ġ
//    Node* goal;                 // ���� ��ǥ ��ġ
//    AgentState state;           // ���� ���� (IDLE, GOING_TO_PARK ��)
//    double total_distance_traveled; // �� �̵� �Ÿ� (���� �Ǵܿ�)
//    int charge_timer;           // ���� �ܿ� �ð� (ƽ)
//    int action_timer;           // ����/���� �۾� �ܿ� �ð� (ƽ)
//    AgentDir heading;           // ���� �ٶ󺸴� ����
//    int rotation_wait;          // ȸ�� ��� �ð� (ƽ)
//
//    Pathfinder* pf;             // D* Lite ��� Ž���� �ν��Ͻ�
//
//    int stuck_steps;            // ���������� �������� ���� ���� �� (���� ���� ������)
//    // --- �۾��� ���� ���� ��Ʈ�� ---
//    int metrics_task_active;        // ���� �۾��� ���� ���� ������ Ȱ��ȭ�Ǿ����� ����
//    int metrics_task_start_step;    // �۾� ���� ������ �ùķ��̼� ����
//    double metrics_distance_at_start; // �۾� ���� ������ �� �̵� �Ÿ�
//    int metrics_turns_current;      // ���� �۾� ���� �߻��� ȸ�� ��
//} Agent;
//
//// --- ȸ�� �� �̵� ���� ó�� ---
//// current���� desired�� �̵� �� ��� ��ȭ�� ���� ��� �Ǵ� �̵��� ����
//// 90�� ȸ�� �� TURN_90_WAIT ��ŭ ����ϸ�, ��� ����(DIR_NONE)�� ù �̵��� ��� ����
//static inline void agent_apply_rotation_and_step(Agent* ag, Node* current, Node* desired, Node** out_next) {
//    if (!ag || !current || !out_next) return;
//    *out_next = current;
//    if (!desired || desired == current) return;
//
//    int dx = desired->x - current->x;
//    int dy = desired->y - current->y;
//    AgentDir new_heading = dir_from_delta(dx, dy);
//    if (new_heading == DIR_NONE) return;
//
//    if (ag->heading == DIR_NONE) {
//        ag->heading = new_heading;
//        *out_next = desired;
//        return;
//    }
//
//    int turn_steps = dir_turn_steps(ag->heading, new_heading);
//    if (turn_steps == 1) {
//        ag->rotation_wait = TURN_90_WAIT - 1;
//        ag->heading = new_heading;
//        ag->metrics_turns_current++;
//        return;
//    }
//    ag->heading = new_heading;
//    *out_next = desired;
//}
//
//// ��� ������Ʈ�� �����ϴ� ����ü
//typedef struct {
//    Agent agents[MAX_AGENTS];
//    int total_cars_parked;  // ���� �����忡 ������ ������ �� ��
//} AgentManager;
//
//// �ó����� ���� ���¸� �����ϴ� ����ü
//typedef struct {
//    SimulationMode mode;            // �ùķ��̼� ��� (����� ����, �ǽð�)
//    int time_step;                  // ���� �ùķ��̼� �ð� (ƽ)
//    int simulation_speed;           // �ùķ��̼� �ӵ� (���� �� ��� �ð� ms)
//    float speed_multiplier;         // �ӵ� ����
//    DynamicPhase phases[MAX_PHASES]; // ����� ���� �ó������� �ܰ� ���
//    int num_phases;                 // �� �ܰ� ��
//    int current_phase_index;        // ���� ���� ���� �ܰ� �ε���
//    int tasks_completed_in_phase;   // ���� �ܰ迡�� �Ϸ�� �۾� ��
//    TaskNode* task_queue_head;      // �ǽð� ����� �۾� ��⿭ ���
//    TaskNode* task_queue_tail;      // �ǽð� ����� �۾� ��⿭ ����
//    int task_count;                 // ��⿭�� �ִ� �� �۾� ��
//    int park_chance;                // �ǽð� ��忡�� ���� ��û�� �߻��� Ȯ�� (0-100)
//    int exit_chance;                // �ǽð� ��忡�� ���� ��û�� �߻��� Ȯ�� (0-100)
//
//
//} ScenarioManager;
//
//// �α� �޽����� �����ϴ� ����ü (��ȯ ����)
//typedef struct {
//    char logs[LOG_BUFFER_LINES][LOG_BUFFER_WIDTH];
//    int log_head;       // ���� ������ �α��� �ε���
//    int log_count;      // ���� ����� �α��� ��
//} Logger;
//
//// Simulation ����ü ���� ���� (������ �Ļ�� ���� �Լ� ���̺� �ñ״�ó��)
//struct Simulation_;
//
//// --- �˰��� ��Ÿ�� ��Ʈ�� ������ (g_metrics�� �̷�) ---
//typedef struct {
//    int whca_h;
//    int wf_edges_last;
//    long long wf_edges_sum;
//    int scc_last;
//    long long scc_sum;
//    int cbs_ok_last;
//    int cbs_exp_last;
//    long long cbs_success_sum;
//    long long cbs_fail_sum;
//} AlgoRTMetrics;
//
//// --- �÷��� ���� (���� ����) ---
//// �ٸ� ��� ��ȹ �˰������� ���� ��ü�� �� �ֵ��� �Լ� �����͸� ���
//typedef struct PlannerVTable {
//    void (*plan_step)(AgentManager*, GridMap*, Logger*, Node* next_pos[MAX_AGENTS]);
//} PlannerVTable;
//
//typedef struct Planner {
//    PlannerVTable vtbl;
//} Planner;
//
//// --- ������ �Ļ�� (�Ļ�� ����) ---
//// ������ ������ ĸ��ȭ�ϰ� �ܼ��� �������̽��� ����
//typedef struct RendererFacadeVTable {
//    void (*draw_frame)(struct Simulation_*, int is_paused);
//} RendererFacadeVTable;
//
//typedef struct RendererFacade_ {
//    RendererFacadeVTable vtbl;
//} RendererFacade;
//
//// �ùķ��̼��� ��� ���¸� �����ϴ� �ֻ��� ����ü
//typedef struct Simulation_ {
//    GridMap* map;
//    AgentManager* agent_manager;
//    ScenarioManager* scenario_manager;
//    Logger* logger;
//    int map_id;                             // ���� ���õ� �� ��ȣ(1~5)
//    PathAlgo path_algo;                     // ���õ� ��� ��ȹ �˰���
//    Planner planner;                        // ���� ���Ͽ� ���� ���� �÷���
//    RendererFacade renderer;                // �������� ����ϴ� �Ļ��
//    int whca_horizon_shadow;                // ���� ȣ������ ���� ���纻 (������)
//    AlgoRTMetrics algo_rt_metrics_shadow;   // g_metrics�� ���纻 (����/�����)
//    double total_cpu_time_ms;               // �ùķ��̼� �� CPU �ð� (ms)
//    double last_step_cpu_time_ms;           // ������ ������ CPU �ð� (ms)
//    double max_step_cpu_time_ms;            // ���� ���� �ɸ� ������ CPU �ð� (ms)
//    double phase_cpu_time_ms[MAX_PHASES];   // �� �ܰ躰 ���� CPU �ð�
//    int phase_step_counts[MAX_PHASES];      // �� �ܰ躰 �� ���� ��
//    int phase_first_step[MAX_PHASES];       // �� �ܰ谡 ���۵� ���� ��ȣ
//    int phase_last_step[MAX_PHASES];        // �� �ܰ谡 ����� ���� ��ȣ
//    int phase_completed_tasks[MAX_PHASES];  // �� �ܰ迡�� �Ϸ�� �۾� ��
//    double post_phase_cpu_time_ms;          // ��� �ܰ� �Ϸ� �� ���� �ܰ��� CPU �ð�
//    int post_phase_step_count;              // ���� �ܰ��� ���� ��
//    int post_phase_first_step;              // ���� �ܰ� ���� ���� ��ȣ
//    int post_phase_last_step;               // ���� �ܰ� ���� ���� ��ȣ
//    double total_planning_time_ms;          // �� ��� ��ȹ �ð�
//    double last_planning_time_ms;           // ������ ������ ��� ��ȹ �ð�
//    double max_planning_time_ms;            // ���� ���� �ɸ� ��� ��ȹ �ð�
//    unsigned long long tasks_completed_total; // �ùķ��̼� ��ü���� �Ϸ�� �۾� ��
//    unsigned long long algorithm_operation_count; // �˰��� ���� Ƚ�� (WFG ����, SCC, CBS Ȯ�� ��)
//    double total_movement_cost;             // ��� ������Ʈ�� �� �̵� �Ÿ�
//    unsigned long long deadlock_count;      // ���� ���·� �����Ǵ� ���� ��
//    double memory_usage_sum_kb;             // �޸� ��뷮 ���� �հ� (KB)
//    double memory_usage_peak_kb;            // �ִ� �޸� ��뷮 (KB)
//    int memory_samples;                     // �޸� ���� Ƚ��
//    // --- �˰��� ���� �޸� ��Ʈ�� ---
//    double algo_mem_sum_kb;                 // �˰��� �ܰ� ���� �޸� ��뷮 ���� �հ�
//    double algo_mem_peak_kb;                // �˰��� �ܰ� ���� �ִ� �޸� ��뷮
//    int algo_mem_samples;                   // �˰��� �޸� ���� Ƚ��
//    int last_task_completion_step;          // ������ �۾��� �Ϸ�� ���� ��ȣ
//    int total_executed_steps;               // �� ����� ������ ���� ��
//    unsigned long long last_report_completed_tasks; // ������ ��ú��� ���� ������ �Ϸ�� �۾� ��
//    int last_report_step;                   // ������ ��ú��� ���� ������ ����
//    // --- �۾� �Ϸ� ���� ���� ��Ʈ�� ---
//    unsigned long long metrics_task_count;  // ������ ������ �� �۾� ��
//    double metrics_sum_dmove;               // �۾��� �̵� �Ÿ� �հ�
//    long long metrics_sum_turns;            // �۾��� ȸ�� �� �հ�
//    double metrics_sum_ttask;               // �۾��� �ҿ� �ð�(����) �հ�
//    // --- �ǽð� ��� ��û ��Ʈ�� ---
//    unsigned long long requests_created_total; // �� ������ ��û ��
//    unsigned long long request_wait_ticks_sum; // ��� ��û�� �� ��� �ð�(ƽ) �հ�
//} Simulation;
//
//// MetricsSnapshot ����ü ���� ���� (������ �ñ״�ó��)
//typedef struct MetricsSnapshot_ MetricsSnapshot;
//
//// Simulation�� Metrics Observer�� �����Ͽ� �����츦 ����ȭ (���Ǵ� MetricsSnapshot ���� ����)
//
//// --- ��Ʈ�� ���� ���� (�۾��� ������) ---
//// Ȱ��ȭ�� �۾��� ������ ��Ʈ���� �����ϰ� ����
//static inline void metrics_finalize_task_if_active(Simulation* sim, Agent* ag) {
//    if (!sim || !ag || !ag->metrics_task_active) return;
//    int steps_now = (sim->scenario_manager ? sim->scenario_manager->time_step : 0);
//    double d_move = ag->total_distance_traveled - ag->metrics_distance_at_start;
//    int turns = ag->metrics_turns_current;
//    double t_task = (double)(steps_now - ag->metrics_task_start_step);
//    if (t_task < 0) t_task = 0;
//    sim->metrics_task_count++;
//    sim->metrics_sum_dmove += d_move;
//    sim->metrics_sum_turns += turns;
//    sim->metrics_sum_ttask += t_task;
//    ag->metrics_task_active = 0;
//    ag->metrics_turns_current = 0;
//}
//
//// --- �ܼ� ��Ʈ�� (���� �г� ǥ�ÿ�) ---
//static struct {
//    int whca_h;
//    int wf_edges_last;
//    long long wf_edges_sum;
//    int scc_last;
//    long long scc_sum;
//    int cbs_ok_last;
//    int cbs_exp_last;
//    long long cbs_success_sum;
//    long long cbs_fail_sum;
//} g_metrics = { 0 };
//
//// --- GlobalConfig (non-invasive; mirrors existing globals by pointer) ---
//typedef struct {
//    int* whca_horizon_ptr;
//    char* display_buffer_ptr;
//} GlobalConfig;
//static GlobalConfig g_config;
//static inline void GlobalConfig_init(GlobalConfig* cfg) {
//    if (!cfg) return;
//    cfg->whca_horizon_ptr = &g_whca_horizon;
//    cfg->display_buffer_ptr = g_display_buf;
//}
//
//// --- Metrics Observer (publish g_metrics snapshots to observers) ---
//typedef struct MetricsSnapshot_ {
//    int whca_h;
//    int wf_edges_last;
//    long long wf_edges_sum;
//    int scc_last;
//    long long scc_sum;
//    int cbs_ok_last;
//    int cbs_exp_last;
//    long long cbs_success_sum;
//    long long cbs_fail_sum;
//    int whca_horizon;
//} MetricsSnapshot;
//
//typedef void (*MetricsObserverFn)(void* ctx, const MetricsSnapshot* snap);
//typedef struct { MetricsObserverFn fn; void* ctx; } MetricsObserver;
//
//#ifndef MAX_METRICS_OBSERVERS
//#define MAX_METRICS_OBSERVERS 8
//#endif
//static MetricsObserver g_metrics_observers[MAX_METRICS_OBSERVERS];
//static int g_metrics_observer_count = 0;
//
//static inline MetricsSnapshot metrics_build_snapshot(void) {
//    MetricsSnapshot s;
//    s.whca_h = g_metrics.whca_h;
//    s.wf_edges_last = g_metrics.wf_edges_last;
//    s.wf_edges_sum = g_metrics.wf_edges_sum;
//    s.scc_last = g_metrics.scc_last;
//    s.scc_sum = g_metrics.scc_sum;
//    s.cbs_ok_last = g_metrics.cbs_ok_last;
//    s.cbs_exp_last = g_metrics.cbs_exp_last;
//    s.cbs_success_sum = g_metrics.cbs_success_sum;
//    s.cbs_fail_sum = g_metrics.cbs_fail_sum;
//    s.whca_horizon = g_whca_horizon;
//    return s;
//}
//static void metrics_subscribe(MetricsObserverFn fn, void* ctx) {
//    if (!fn) return;
//    if (g_metrics_observer_count >= MAX_METRICS_OBSERVERS) return;
//    g_metrics_observers[g_metrics_observer_count++] = (MetricsObserver){ fn, ctx };
//}
//static void metrics_notify_all(void) {
//    MetricsSnapshot snap = metrics_build_snapshot();
//    for (int i = 0; i < g_metrics_observer_count; i++) {
//        if (g_metrics_observers[i].fn) g_metrics_observers[i].fn(g_metrics_observers[i].ctx, &snap);
//    }
//}
//
//// Simulation�� Metrics Observer�� �����Ͽ� �����츦 ����ȭ
//static void simulation_metrics_observer(void* ctx, const MetricsSnapshot* s) {
//    Simulation* sim = (Simulation*)ctx;
//    if (!sim || !s) return;
//    sim->whca_horizon_shadow = s->whca_horizon;
//    sim->algo_rt_metrics_shadow.whca_h = s->whca_h;
//    sim->algo_rt_metrics_shadow.wf_edges_last = s->wf_edges_last;
//    sim->algo_rt_metrics_shadow.wf_edges_sum = s->wf_edges_sum;
//    sim->algo_rt_metrics_shadow.scc_last = s->scc_last;
//    sim->algo_rt_metrics_shadow.scc_sum = s->scc_sum;
//    sim->algo_rt_metrics_shadow.cbs_ok_last = s->cbs_ok_last;
//    sim->algo_rt_metrics_shadow.cbs_exp_last = s->cbs_exp_last;
//    sim->algo_rt_metrics_shadow.cbs_success_sum = s->cbs_success_sum;
//    sim->algo_rt_metrics_shadow.cbs_fail_sum = s->cbs_fail_sum;
//}
//
//// --- ȣ������ ���� ���� ---
//static int g_conflict_score = 0;
//
//// --- ������ (��ü ����: ������ ���� ���� ������ ����ü�� ����) ---
//typedef struct {
//    int render_stride;
//    int fast_render;
//    int simple_colors;
//} Renderer;
//
//static Renderer g_renderer = { 1, 0, 0 };
//
//// =============================================================================
//// ���� 3: �Լ� ������Ÿ��
//// =============================================================================
//// --- API ȣȯ�� ��Ī (���ı���) ---
//#define grid_map_create Grid_create
//#define grid_map_destroy Grid_destroy
//#define grid_is_valid_coord Grid_isValidCoord
//#define grid_is_node_blocked Grid_isNodeBlocked
//
//void ensure_console_width(int minCols);
//
//Simulation* simulation_create();
//void simulation_destroy(Simulation* sim);
//void simulation_run(Simulation* sim);
//void simulation_print_performance_summary(const Simulation* sim);
//static void simulation_report_realtime_dashboard(Simulation* sim);
//static void simulation_collect_memory_sample(Simulation* sim);
//static void simulation_collect_memory_sample_algo(Simulation* sim);
//static void simulation_reset_runtime_stats(Simulation* sim);
//// UI helpers (unified naming)
//static void ui_append_controls_help(char** p, size_t* rem);
//static void ui_flush_display_buffer(void);
//// Control/Plan helpers (unified naming)
//static void ui_handle_control_key(Simulation* sim, int ch, int* is_paused, int* quit_flag);
//static void simulation_plan_step(Simulation* sim, Node* next_pos[MAX_AGENTS]);
//static int apply_moves_and_update_stuck(Simulation* sim, Node* next_pos[MAX_AGENTS], Node* prev_pos[MAX_AGENTS]);
//static void update_deadlock_counter(Simulation* sim, int moved_this_step, int is_custom_mode);
//static void accumulate_wait_ticks_if_realtime(Simulation* sim);
//static void maybe_report_realtime_dashboard(Simulation* sim);
//// Planner strategy helper
//static Planner planner_from_pathalgo(PathAlgo algo);
//// Renderer facade helper
//static RendererFacade renderer_create_facade(void);
//// One-step execution helper (encapsulate one simulation tick)
//static void simulation_execute_one_step(Simulation* sim, int is_paused);
//// Cleanup helper
//static void force_idle_cleanup(AgentManager* am, Simulation* sim, Logger* lg);
//// Agent/AgentManager OO-like wrappers
//void agent_begin_task_park(Agent* ag, ScenarioManager* sc, Logger* lg);
//void agent_begin_task_exit(Agent* ag, ScenarioManager* sc, Logger* lg);
//
//void system_enable_virtual_terminal();
//void ui_clear_screen_optimized();
//
//int simulation_setup(Simulation* sim);
//
//// �� Map selection
//static int simulation_setup_map(Simulation* sim);
//void grid_map_load_scenario(GridMap* map, AgentManager* am, int scenario_id);
//// --- Procedural map builders (for Map #2 ~ #5)
//static void map_build_hypermart(GridMap* m, AgentManager* am);       // #2
//static void map_build_10agents_200slots(GridMap* m, AgentManager* am); // #3
//static void map_build_biggrid_onegoal(GridMap* m, AgentManager* am); // #4
//static void map_build_cross_4agents(GridMap* m, AgentManager* am);   // #5 Cross map
//
//// Common helpers
//static void grid_map_clear(GridMap* map);
//static void map_all_free(GridMap* m);
//static void map_add_border_walls(GridMap* m);
//static void map_place_goal(GridMap* m, int x, int y);
//static void map_place_charge(GridMap* m, int x, int y);
//static void map_place_agent_at(AgentManager* am, GridMap* m, int idx, int x, int y);
//static void map_reserve_area_as_start(GridMap* m, int x0, int y0, int w, int h); // ����
//
//
//
//// Logger
//void logger_log(Logger* logger, const char* format, ...);
//Logger* logger_create();
//void logger_destroy(Logger*);
//
//// Grid
//GridMap* grid_map_create(AgentManager*);
//void grid_map_destroy(GridMap*);
//int grid_is_valid_coord(int x, int y);
//int grid_is_node_blocked(const GridMap*, const AgentManager*, const Node*, const struct Agent_*);
//
//// Scenario
//ScenarioManager* scenario_manager_create();
//void scenario_manager_destroy(ScenarioManager*);
//
//// Agents
//AgentManager* agent_manager_create();
//void agent_manager_destroy(AgentManager*);
//void agent_manager_plan_and_resolve_collisions(AgentManager*, GridMap*, Logger*, Node* next_pos[MAX_AGENTS]);
//void agent_manager_update_state_after_move(AgentManager*, ScenarioManager*, GridMap*, Logger*, Simulation*);
//void agent_manager_update_charge_state(AgentManager*, GridMap*, Logger*);
//
//// Alternate planners (for algorithm selection)
//void agent_manager_plan_and_resolve_collisions_astar(AgentManager*, GridMap*, Logger*, Node* next_pos[MAX_AGENTS]);
//void agent_manager_plan_and_resolve_collisions_dstar_basic(AgentManager*, GridMap*, Logger*, Node* next_pos[MAX_AGENTS]);
//
//// Pathfinder (Incremental D* Lite)
//Pathfinder* pathfinder_create(Node* start, Node* goal, const struct Agent_* agent);
//void pathfinder_destroy(Pathfinder* pf);
//void pathfinder_reset_goal(Pathfinder* pf, Node* new_goal);
//void pathfinder_update_start(Pathfinder* pf, Node* new_start);
//void pathfinder_notify_cell_change(Pathfinder* pf, GridMap* map, const AgentManager* am, Node* changed);
//void pathfinder_compute_shortest_path(Pathfinder* pf, GridMap* map, const AgentManager* am);
//Node* pathfinder_get_next_step(Pathfinder* pf, const GridMap* map, const AgentManager* am, Node* current_node);
//
//// WFG + CBS helpers
//static void add_wait_edge(WaitEdge* edges, int* cnt, int from, int to, int t, CauseType cause, int x1, int y1, int x2, int y2);
//static int  build_scc_mask_from_edges(const WaitEdge* edges, int cnt);
//static int  run_partial_CBS(AgentManager* m, GridMap* map, Logger* lg,
//    int group_ids[], int group_n, const ReservationTable* base_rt,
//    Node* out_plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1]);
//
//// D* Lite ���� ��ε�ĳ��Ʈ
//static void broadcast_cell_change(AgentManager* am, GridMap* map, Node* changed);
//
//// ȣ������ �ڵ�����
//static void WHCA_adjustHorizon(int wf_edges, int scc, Logger* lg);
//
//// �Է� ����
//static char get_single_char();
//static char get_char_input(const char* prompt, const char* valid);
//static int  get_integer_input(const char* prompt, int min, int max);
//static float get_float_input(const char* prompt, float min, float max);
//// *** NEW *** Non-blocking input check
//static int check_for_input();
//
//// --- PathfinderFactory: ������δ� ����/�ı� ���� ---
//typedef struct PathfinderFactory_ {
//    Pathfinder* (*create)(Node* start, Node* goal);
//    void (*destroy)(Pathfinder* pf);
//} PathfinderFactory;
//static inline Pathfinder* pf_factory_create(Node* start, Node* goal) { return pathfinder_create(start, goal, NULL); }
//static inline void pf_factory_destroy(Pathfinder* pf) { pathfinder_destroy(pf); }
//static PathfinderFactory g_pf_factory = { pf_factory_create, pf_factory_destroy };
//
//// ���� �浹 �ؼ� ���� ����
//static void resolve_conflicts_by_order(const AgentManager* m, const int order[MAX_AGENTS], Node* next_pos[MAX_AGENTS]);
//
//
//// =============================================================================
//// ���� 4: �ý���, ����� �� ���� �Լ�
//// =============================================================================
//// --- D* Lite ������ ������ �ӽ� ��ŷ ���� (���� Ž�� �ϰ��� ����) ---
///**
// * @brief l->nodes�� ��ϵ� ��� �ӽ� ��ֹ� ��ŷ�� �����ϰ�,
// *        D* Lite ��� Ž���⿡ ȯ�� ��ȭ(�� ��� ����)�� �����մϴ�.
// * @param l �ӽ� ��ŷ�� ��� ���
// * @param pf ������ ������ Pathfinder �ν��Ͻ�
// * @param map �׸��� ��
// * @param am ������Ʈ ������
// */
//static inline void temp_unmark_all_and_notify(TempMarkList* l, Pathfinder* pf, GridMap* map, const AgentManager* am) {
//    if (!l) return;
//    for (int i = 0; i < l->count; i++) {
//        Node* n = l->nodes[i];
//        if (!n) continue;
//        n->is_temp = FALSE;
//        if (pf) pathfinder_notify_cell_change(pf, map, am, n);
//    }
//    l->count = 0;
//}
//// --- ���� �ӽ� ��ŷ ���ؽ�Ʈ (D* Lite �ڵ� ���� �ɼ� ����) ---
//typedef struct {
//    TempMarkList marks;     // �ӽ� ��ŷ�� ��� ���
//    Pathfinder* pf;         // (����) �ڵ� ���� Ȱ��ȭ �� ���� Pathfinder
//    GridMap* map;           // (�ڵ� ���� �� �ʼ�) �׸��� ��
//    AgentManager* am;       // (�ڵ� ���� �� �ʼ�) ������Ʈ ������
//    int auto_notify;        // 0: ��ŷ ������, 1: ���� + D* Lite ����
//} TempMarkContext;
//
///**
// * @brief �ӽ� ��ŷ ���ؽ�Ʈ�� �ʱ�ȭ�մϴ�.
// * @param ctx �ʱ�ȭ�� ���ؽ�Ʈ ������
// * @param pf (����) �ڵ� ������ ����� Pathfinder
// * @param map (�ڵ� ���� �� �ʼ�) �׸��� ��
// * @param am (�ڵ� ���� �� �ʼ�) ������Ʈ ������
// * @param auto_notify �ڵ� ���� Ȱ��ȭ ����
// */
//static inline void temp_context_init(TempMarkContext* ctx, Pathfinder* pf, GridMap* map, AgentManager* am, int auto_notify) {
//    if (!ctx) return;
//    temp_mark_init(&ctx->marks);
//    ctx->pf = pf;
//    ctx->map = map;
//    ctx->am = am;
//    ctx->auto_notify = auto_notify;
//}
//
///**
// * @brief ���ؽ�Ʈ�� ���� ��带 �ӽ� ��ֹ��� ��ŷ�մϴ�.
// *        auto_notify�� Ȱ��ȭ�� ���, D* Lite�� ��� ���� ������ �����մϴ�.
// * @param ctx �ӽ� ��ŷ ���ؽ�Ʈ
// * @param n ��ŷ�� ���
// */
//static inline void temp_context_mark(TempMarkContext* ctx, Node* n) {
//    if (!ctx || !n) return;
//    temp_mark_node(&ctx->marks, n);
//    if (ctx->auto_notify && ctx->pf) {
//        pathfinder_notify_cell_change(ctx->pf, ctx->map, ctx->am, n);
//    }
//}
//
///**
// * @brief ���ؽ�Ʈ�� ��ϵ� ��� �ӽ� ��ŷ�� �����մϴ�.
// *        auto_notify ������ ���� D* Lite ���� ���ΰ� �����˴ϴ�.
// * @param ctx ������ ���ؽ�Ʈ
// */
//static inline void temp_context_cleanup(TempMarkContext* ctx) {
//    if (!ctx) return;
//    if (ctx->auto_notify) temp_unmark_all_and_notify(&ctx->marks, ctx->pf, ctx->map, ctx->am);
//    else temp_unmark_all(&ctx->marks);
//}
///**
// * @brief Windows �ֿܼ��� ���� �͹̳�(ANSI �̽������� ������) ó���� Ȱ��ȭ�մϴ�.
// *        �̸� ���� ���� �ڵ�, Ŀ�� �̵� �� �پ��� �͹̳� ��� ���������ϴ�.
// */
//void system_enable_virtual_terminal() {
//    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
//    if (hOut == INVALID_HANDLE_VALUE) return;
//    DWORD dwMode = 0;
//    if (!GetConsoleMode(hOut, &dwMode)) return;
//    dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
//    SetConsoleMode(hOut, dwMode);
//}
//
///**
// * @brief ȭ���� ����ϴ�. ���� �͹̳� ���� ���ο� ���� ����ȭ�� ����� ����մϴ�.
// *        - ���� ��: ANSI �̽������� ������(\\x1b[2J)�� ����Ͽ� ȭ��� ��ũ�ѹ� ���۸� ��� ����ϴ�.
// *        - ������ ��: ���� Windows API�� ����Ͽ� ȭ�� ���� ��ü�� �������� ä��ϴ�.
// */
//void ui_clear_screen_optimized() {
//    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
//    if (hConsole == INVALID_HANDLE_VALUE) return;
//
//    DWORD mode = 0;
//    if (GetConsoleMode(hConsole, &mode) && (mode & ENABLE_VIRTUAL_TERMINAL_PROCESSING)) {
//        // VT(ANSI) ������: ȭ��+��ũ�ѹ� ��� ����� Ŀ�� Ȩ
//        fputs("\x1b[H\x1b[2J\x1b[3J", stdout);
//        fflush(stdout);
//        return;
//    }
//
//    // (���Ž� CMD ��) VT ������: ���� Win32 ������� ���� ��ü �����
//    CONSOLE_SCREEN_BUFFER_INFO csbi;
//    if (!GetConsoleScreenBufferInfo(hConsole, &csbi)) return;
//
//    DWORD cellCount = (DWORD)csbi.dwSize.X * (DWORD)csbi.dwSize.Y;
//    DWORD count;
//    COORD home = { 0, 0 };
//
//    FillConsoleOutputCharacterA(hConsole, ' ', cellCount, home, &count);
//    FillConsoleOutputAttribute(hConsole, csbi.wAttributes, cellCount, home, &count);
//    SetConsoleCursorPosition(hConsole, home);
//}
//
///**
// * @brief �ܼ� â�� �ʺ� �ּ� �ʺ�(minCols)���� ���� ���, ���� ũ�⸦ �����Ͽ�
// *        �� �ٲ� ������ �����ϰ� UI�� ������ �ʵ��� �մϴ�.
// * @param minCols �ʿ��� �ּ� �ܼ� �ʺ�
// */
//void ensure_console_width(int minCols) {
//    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
//    if (h == INVALID_HANDLE_VALUE) return;
//
//    CONSOLE_SCREEN_BUFFER_INFO csbi;
//    if (!GetConsoleScreenBufferInfo(h, &csbi)) return;
//
//    COORD size = csbi.dwSize;
//    if (size.X < minCols) {
//        size.X = (SHORT)minCols;
//        // ���� ���� ���� Ű���ָ� â ũ�� ���� ���� �ٰ����� �پ��ϴ�.
//        SetConsoleScreenBufferSize(h, size);
//    }
//}
//
///**
// * @brief ���� ���μ����� �޸� ��뷮(Working Set)�� �����Ͽ� Simulation ����ü�� ����մϴ�.
// * @param sim �޸� ��뷮 �����͸� ������ Simulation �ν��Ͻ�
// */
//static void simulation_collect_memory_sample(Simulation* sim) {
//#ifdef _WIN32
//    PROCESS_MEMORY_COUNTERS pmc;
//    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
//        double working_set_kb = (double)pmc.WorkingSetSize / 1024.0;
//        sim->memory_usage_sum_kb += working_set_kb;
//        if (working_set_kb > sim->memory_usage_peak_kb) sim->memory_usage_peak_kb = working_set_kb;
//        sim->memory_samples++;
//    }
//#else
//    (void)sim;
//#endif
//}
//
///**
// * @brief �˰��� ���� ������ �޸� ��뷮�� �����մϴ�.
// *        �̴� ������, �α� ��� �� �ٸ� ����� ������ �ּ�ȭ�ϰ�
// *        ���� �˰����� �޸� ��뷮�� �����ϱ� �����Դϴ�.
// * @param sim �޸� ��뷮 �����͸� ������ Simulation �ν��Ͻ�
// */
//static void simulation_collect_memory_sample_algo(Simulation* sim) {
//#ifdef _WIN32
//    PROCESS_MEMORY_COUNTERS pmc;
//    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
//        double working_set_kb = (double)pmc.WorkingSetSize / 1024.0;
//        sim->algo_mem_sum_kb += working_set_kb;
//        if (working_set_kb > sim->algo_mem_peak_kb) sim->algo_mem_peak_kb = working_set_kb;
//        sim->algo_mem_samples++;
//    }
//#else
//    (void)sim;
//#endif
//}
//
///**
// * @brief �ùķ��̼��� ��Ÿ�� ��� ������(CPU �ð�, �޸� ��뷮, �۾� �� ��)�� ��� �ʱ�ȭ�մϴ�.
// *        ���ο� �ùķ��̼� ���� ���� ȣ��˴ϴ�.
// * @param sim �ʱ�ȭ�� Simulation �ν��Ͻ�
// */
//static void simulation_reset_runtime_stats(Simulation* sim) {
//    if (!sim) return;
//    sim->total_cpu_time_ms = 0.0;
//    sim->last_step_cpu_time_ms = 0.0;
//    sim->max_step_cpu_time_ms = 0.0;
//    memset(sim->phase_cpu_time_ms, 0, sizeof(sim->phase_cpu_time_ms));
//    memset(sim->phase_step_counts, 0, sizeof(sim->phase_step_counts));
//    memset(sim->phase_completed_tasks, 0, sizeof(sim->phase_completed_tasks));
//    for (int i = 0; i < MAX_PHASES; i++) {
//        sim->phase_first_step[i] = -1;
//        sim->phase_last_step[i] = -1;
//    }
//    sim->post_phase_cpu_time_ms = 0.0;
//    sim->post_phase_step_count = 0;
//    sim->post_phase_first_step = -1;
//    sim->post_phase_last_step = -1;
//    sim->total_planning_time_ms = 0.0;
//    sim->last_planning_time_ms = 0.0;
//    sim->max_planning_time_ms = 0.0;
//    sim->tasks_completed_total = 0;
//    sim->algorithm_operation_count = 0;
//    sim->total_movement_cost = 0.0;
//    sim->deadlock_count = 0;
//    sim->memory_usage_sum_kb = 0.0;
//    sim->memory_usage_peak_kb = 0.0;
//    sim->memory_samples = 0;
//    sim->algo_mem_sum_kb = 0.0;
//    sim->algo_mem_peak_kb = 0.0;
//    sim->algo_mem_samples = 0;
//    sim->last_task_completion_step = 0;
//    sim->total_executed_steps = 0;
//    sim->last_report_completed_tasks = 0;
//    sim->last_report_step = 0;
//    sim->metrics_task_count = 0;
//    sim->metrics_sum_dmove = 0.0;
//    sim->metrics_sum_turns = 0;
//    sim->metrics_sum_ttask = 0.0;
//    sim->requests_created_total = 0;
//    sim->request_wait_ticks_sum = 0;
//}
//
///**
// * @brief �ǽð� ��忡�� �ֱ������� ���� �ùķ��̼� ���� ��ú��带 �ֿܼ� ����մϴ�.
// * @param sim ��ú��� ������ ������ Simulation �ν��Ͻ�
// */
//static void simulation_report_realtime_dashboard(Simulation* sim) {
//    ScenarioManager* sc = sim->scenario_manager;
//    int steps = (sim->total_executed_steps > 0) ? sim->total_executed_steps : (sc ? sc->time_step : 0);
//    if (steps <= 0) steps = 1;
//
//    unsigned long long total_completed = sim->tasks_completed_total;
//    int interval_steps = steps - sim->last_report_step;
//    if (interval_steps <= 0) interval_steps = 1;
//    unsigned long long delta_completed = total_completed - sim->last_report_completed_tasks;
//
//    double throughput_avg = (double)total_completed / (double)steps;
//    double throughput_interval = (double)delta_completed / (double)interval_steps;
//    double avg_planning_ms = (steps > 0) ? sim->total_planning_time_ms / (double)steps : 0.0;
//    double avg_memory_kb = (sim->memory_samples > 0) ? sim->memory_usage_sum_kb / (double)sim->memory_samples : 0.0;
//
//    int active_agents = 0;
//    if (sim->agent_manager) {
//        for (int i = 0; i < MAX_AGENTS; i++) {
//            if (sim->agent_manager->agents[i].pos) active_agents++;
//        }
//    }
//
//    printf("\n========== Real-Time Dashboard @ step %d ==========\n", steps);
//    printf(" Total Physical Time Steps      : %d\n", steps);
//    printf(" Operating AGVs                 : %d\n", active_agents);
//    printf(" Tasks Completed (total)        : %llu\n", total_completed);
//    printf(" Throughput (total avg)         : %.4f tasks/step\n", throughput_avg);
//    printf(" Throughput (last interval)     : %.4f tasks/step over %d steps\n", throughput_interval, interval_steps);
//    printf(" Total Computation CPU Time     : %.2f ms\n", sim->total_cpu_time_ms);
//    printf(" Average Planning Time / Step   : %.4f ms\n", avg_planning_ms);
//    printf(" Total Task Completion Step     : %d\n", sim->last_task_completion_step);
//    printf(" Total Movement Cost            : %.2f cells\n", sim->total_movement_cost);
//    printf(" Requests Created (total)       : %llu\n", sim->requests_created_total);
//    printf(" Request Wait Ticks (sum)       : %llu\n", sim->request_wait_ticks_sum);
//    printf(" Process Memory Usage Sum      : %.2f KB (avg %.2f KB / sample, peak %.2f KB)\n",
//        sim->memory_usage_sum_kb, avg_memory_kb, sim->memory_usage_peak_kb);
//    printf("===================================================\n");
//
//    sim->last_report_completed_tasks = total_completed;
//    sim->last_report_step = steps;
//}
//
//
///**
// * @brief Ű���� �Է��� �ִ��� �񵿱������� Ȯ���մϴ�.
// * @return �Է��� ������ �ش� Ű�� ASCII �ڵ带, ������ 0�� ��ȯ�մϴ�.
// */
//static int check_for_input() {
//    if (_kbhit()) {
//        return _getch();
//    }
//    return 0;
//}
//
//// --- ���� ���� ĸ��ȭ ---
//typedef struct {
//    int is_paused;  // �Ͻ����� ����
//    int quit_flag;  // ���� �÷���
//    int last_key;   // ���������� �Էµ� Ű
//} ControlState;
///**
// * @brief ControlState ����ü�� �⺻������ �ʱ�ȭ�մϴ�.
// * @param cs �ʱ�ȭ�� ControlState ������
// */
//static inline void ControlState_init(ControlState* cs) {
//    if (!cs) return;
//    cs->is_paused = FALSE;
//    cs->quit_flag = FALSE;
//    cs->last_key = 0;
//}
//
///**
// * @brief �ֿܼ��� ���� ���ڸ� �Է¹޽��ϴ�. (���� ����)
// * @return �Էµ� ������ ASCII �ڵ�
// */
//static char get_single_char() {
//    return _getch();
//}
//
///**
// * @brief ����ڿ��� ������Ʈ�� ǥ���ϰ�, ��ȿ�� ���� ����(valid) �� �ϳ��� �Է¹��� ������ �ݺ��մϴ�.
// * @param prompt ����ڿ��� ������ �ȳ� �޽���
// * @param valid �Է� ������ ��ȿ�� ���ڵ�� �̷���� ���ڿ�
// * @return ����ڰ� �Է��� ��ȿ�� ���� (�ҹ��ڷ� ��ȯ��)
// */
//static char get_char_input(const char* prompt, const char* valid) {
//    char c;
//    while (TRUE) {
//        printf("%s", prompt);
//        c = (char)tolower(get_single_char());
//        printf("%c\n", c);
//        if (strchr(valid, c)) return c;
//        printf(C_B_RED "\n�߸��� �Է��Դϴ�. (%s)\n" C_NRM, valid);
//    }
//}
//
///**
// * @brief ����ڿ��� ������Ʈ�� ǥ���ϰ�, ������ ���� [min, max] ���� ������ �Է¹޽��ϴ�.
// * @param prompt ����ڿ��� ������ �ȳ� �޽���
// * @param min �Է� ������ �ּ� ������
// * @param max �Է� ������ �ִ� ������
// * @return ����ڰ� �Է��� ��ȿ�� ����
// */
//static int get_integer_input(const char* prompt, int min, int max) {
//    char buf[INPUT_BUFFER_SIZE];
//    int v;
//    while (TRUE) {
//        printf("%s", prompt);
//        if (fgets(buf, sizeof(buf), stdin) && sscanf(buf, "%d", &v) == 1 && v >= min && v <= max) return v;
//        printf(C_B_RED "�߸��� �Է��Դϴ�. %d~%d ����.\n" C_NRM, min, max);
//    }
//}
//
///**
// * @brief ����ڿ��� ������Ʈ�� ǥ���ϰ�, ������ ���� [min, max] ���� �Ǽ��� �Է¹޽��ϴ�.
// * @param prompt ����ڿ��� ������ �ȳ� �޽���
// * @param min �Է� ������ �ּ� �Ǽ���
// * @param max �Է� ������ �ִ� �Ǽ���
// * @return ����ڰ� �Է��� ��ȿ�� �Ǽ�
// */
//static float get_float_input(const char* prompt, float min, float max) {
//    char buf[INPUT_BUFFER_SIZE];
//    float v;
//    while (TRUE) {
//        printf("%s", prompt);
//        if (fgets(buf, sizeof(buf), stdin) && sscanf(buf, "%f", &v) == 1 && v >= min && v <= max) return v;
//        printf(C_B_RED "�߸��� �Է��Դϴ�. %.1f~%.1f.\n" C_NRM, min, max);
//    }
//}
//
//// ---- UI / Render ----
//void ui_enter_alt_screen(void) {
//    // ALT buffer + Ȩ + Ŀ�� ����
//    fputs("\x1b[?1049h\x1b[H\x1b[?25l", stdout);
//    fflush(stdout);
//}
//void ui_leave_alt_screen(void) {
//    // ALT buffer ���� + Ŀ�� ���̱�
//    fputs("\x1b[?1049l\x1b[?25h", stdout);
//    fflush(stdout);
//}
//
//static void ui_append_controls_help(char** p, size_t* rem) {
//    APPEND_FMT(*p, *rem, "%s--- Controls ---%s\n", C_B_WHT, C_NRM);
//    APPEND_FMT(*p, *rem, "[%sP%s]ause/Resume | [%sS%s]tep | [%s+%s]/[%s-%s] Speed | ",
//        C_YEL, C_NRM, C_YEL, C_NRM, C_YEL, C_NRM, C_YEL, C_NRM);
//    APPEND_FMT(*p, *rem, "[%s[%s]/[%s]%s Render stride | ", C_YEL, C_NRM, C_YEL, C_NRM);
//    APPEND_FMT(*p, *rem, "[%sF%s]ast render | [%sC%s]olor simple | [%sQ%s]uit\n",
//        C_YEL, C_NRM, C_YEL, C_NRM, C_YEL, C_NRM);
//}
//
//static void ui_flush_display_buffer(void) {
//    size_t cur_len = strlen(g_display_buf);
//    if (!g_renderer.fast_render) ui_clear_screen_optimized(); else fputs("\x1b[H", stdout);
//    fwrite(g_display_buf, 1, cur_len, stdout);
//    fflush(stdout);
//}
//
///**
// * @brief ������� Ű �Է��� �޾� �ùķ��̼� ����(�Ͻ�����, �ӵ�, ���� ��)�� �����մϴ�.
// * @param sim ������ Simulation �ν��Ͻ�
// * @param ch ����ڰ� �Է��� Ű ����
// * @param is_paused �Ͻ����� ���� �÷��� ������
// * @param quit_flag ���� �÷��� ������
// */
//static void ui_handle_control_key(Simulation* sim, int ch, int* is_paused, int* quit_flag) {
//    switch (tolower(ch)) {
//    case 'p':
//        *is_paused = !*is_paused;
//        logger_log(sim->logger, *is_paused ? "[CTRL] Simulation Paused." : "[CTRL] Simulation Resumed.");
//        break;
//    case 's':
//        if (*is_paused) {
//            logger_log(sim->logger, "[CTRL] Advancing one step.");
//        }
//        break;
//    case '+':
//    case '=':
//        sim->scenario_manager->speed_multiplier += 0.5f;
//        if (sim->scenario_manager->speed_multiplier > MAX_SPEED_MULTIPLIER)
//            sim->scenario_manager->speed_multiplier = MAX_SPEED_MULTIPLIER;
//        logger_log(sim->logger, "[CTRL] Speed increased to %.1fx", sim->scenario_manager->speed_multiplier);
//        break;
//    case '-':
//        sim->scenario_manager->speed_multiplier -= 0.5f;
//        if (sim->scenario_manager->speed_multiplier < 0.1f)
//            sim->scenario_manager->speed_multiplier = 0.1f;
//        logger_log(sim->logger, "[CTRL] Speed decreased to %.1fx", sim->scenario_manager->speed_multiplier);
//        break;
//    case 'q':
//        *quit_flag = TRUE;
//        logger_log(sim->logger, "[CTRL] Quit simulation.");
//        break;
//    case ']':
//        if (g_renderer.render_stride < RENDER_STRIDE_MAX) g_renderer.render_stride <<= 1;
//        if (g_renderer.render_stride < RENDER_STRIDE_MIN) g_renderer.render_stride = RENDER_STRIDE_MIN;
//        logger_log(sim->logger, "[CTRL] Render stride = %d", g_renderer.render_stride);
//        break;
//    case '[':
//        if (g_renderer.render_stride > RENDER_STRIDE_MIN) g_renderer.render_stride >>= 1;
//        logger_log(sim->logger, "[CTRL] Render stride = %d", g_renderer.render_stride);
//        break;
//    case 'f':
//        g_renderer.fast_render = !g_renderer.fast_render;
//        logger_log(sim->logger, g_renderer.fast_render ? "[CTRL] Fast render ON" : "[CTRL] Fast render OFF");
//        break;
//    case 'c':
//        g_renderer.simple_colors = !g_renderer.simple_colors;
//        logger_log(sim->logger, g_renderer.simple_colors ? "[CTRL] Simple colors ON" : "[CTRL] Simple colors OFF");
//        break;
//    }
//
//    // �ӵ� ���� �� sleep �ð� ����
//    if (ch == '+' || ch == '=' || ch == '-') {
//        sim->scenario_manager->simulation_speed = (int)(100.0f / sim->scenario_manager->speed_multiplier);
//        if (sim->scenario_manager->simulation_speed < 0) sim->scenario_manager->simulation_speed = 0;
//    }
//}
//
///**
// * @brief ���� ���õ� ��� ��ȹ �˰��� ���� ������Ʈ���� ���� ������ ��ȹ�մϴ�.
// *        ��ȹ�� �ҿ�� CPU �ð��� �����Ͽ� Simulation ��迡 ����մϴ�.
// * @param sim �ùķ��̼� �ν��Ͻ�
// * @param next_pos �� ������Ʈ�� ���� ��ġ�� ������ �迭
// */
//static void simulation_plan_step(Simulation* sim, Node* next_pos[MAX_AGENTS]) {
//    clock_t plan_start_cpu = clock();
//    if (sim->planner.vtbl.plan_step) {
//        sim->planner.vtbl.plan_step(sim->agent_manager, sim->map, sim->logger, next_pos);
//    }
//    else {
//        switch (sim->path_algo) {
//        case PATHALGO_ASTAR_SIMPLE:
//            agent_manager_plan_and_resolve_collisions_astar(sim->agent_manager, sim->map, sim->logger, next_pos);
//            break;
//        case PATHALGO_DSTAR_BASIC:
//            agent_manager_plan_and_resolve_collisions_dstar_basic(sim->agent_manager, sim->map, sim->logger, next_pos);
//            break;
//        case PATHALGO_DEFAULT:
//        default:
//            agent_manager_plan_and_resolve_collisions(sim->agent_manager, sim->map, sim->logger, next_pos);
//            break;
//        }
//    }
//    clock_t plan_end_cpu = clock();
//    double planning_time_ms = ((double)(plan_end_cpu - plan_start_cpu) * 1000.0) / CLOCKS_PER_SEC;
//    sim->last_planning_time_ms = planning_time_ms;
//    sim->total_planning_time_ms += planning_time_ms;
//    if (planning_time_ms > sim->max_planning_time_ms) sim->max_planning_time_ms = planning_time_ms;
//    sim->algorithm_operation_count += (unsigned long long)((g_metrics.wf_edges_last > 0 ? g_metrics.wf_edges_last : 0) +
//        (g_metrics.scc_last > 0 ? g_metrics.scc_last : 0) +
//        (g_metrics.cbs_exp_last > 0 ? g_metrics.cbs_exp_last : 0));
//    // ��Ʈ�� ���� �˸� (Observer ����)
//    metrics_notify_all();
//}
//
///**
// * @brief ��ȹ�� ���� ��ġ(next_pos)�� ������Ʈ�� �����ϰ�, �̵� ���θ� Ȯ���մϴ�.
// *        �̵����� ���� ������Ʈ�� 'stuck_steps' ī���͸� ������ŵ�ϴ�.
// * @param sim �ùķ��̼� �ν��Ͻ�
// * @param next_pos �� ������Ʈ�� ��ȹ�� ���� ��ġ �迭
// * @param prev_pos �� ������Ʈ�� ���� ��ġ �迭
// * @return �� �� �̻��� ������Ʈ�� �̵������� 1, �ƴϸ� 0�� ��ȯ�մϴ�.
// */
//static int apply_moves_and_update_stuck(Simulation* sim, Node* next_pos[MAX_AGENTS], Node* prev_pos[MAX_AGENTS]) {
//    int moved_this_step = 0;
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* ag = &sim->agent_manager->agents[i];
//        if (ag->state != CHARGING && next_pos[i]) {
//            if (ag->pos != next_pos[i]) {
//                ag->total_distance_traveled += 1.0;
//                sim->total_movement_cost += 1.0;
//                moved_this_step = 1;
//            }
//            ag->pos = next_pos[i];
//        }
//    }
//
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* ag = &sim->agent_manager->agents[i];
//        if (ag->state == CHARGING || ag->state == IDLE || ag->action_timer > 0) { ag->stuck_steps = 0; continue; }
//        if (ag->pos == prev_pos[i]) ag->stuck_steps++;
//        else ag->stuck_steps = 0;
//    }
//
//    return moved_this_step;
//}
//
///**
// * @brief �̹� ���ܿ��� �ƹ��� �������� �ʾҰ� �ذ��ؾ� �� �۾��� �����ִ� ���,
// *        ���� ����(deadlock) ī���͸� ������ŵ�ϴ�.
// * @param sim �ùķ��̼� �ν��Ͻ�
// * @param moved_this_step �̹� ���ܿ��� �̵��� ������Ʈ�� �־����� ����
// * @param is_custom_mode ���� �ó������� ����� ���� ������� ����
// */
//static void update_deadlock_counter(Simulation* sim, int moved_this_step, int is_custom_mode) {
//    ScenarioManager* sc = sim->scenario_manager;
//    if (moved_this_step) return;
//    int unresolved = 0;
//    if (is_custom_mode) {
//        if (sc->current_phase_index < sc->num_phases) {
//            const DynamicPhase* ph = &sc->phases[sc->current_phase_index];
//            if (sc->tasks_completed_in_phase < ph->task_count) unresolved = 1;
//        }
//    }
//    else if (sc->mode == MODE_REALTIME) {
//        if (sc->task_count > 0) unresolved = 1;
//    }
//    if (unresolved) sim->deadlock_count++;
//}
//
///**
// * @brief �ǽð� ��忡�� ��⿭�� �۾��� �ִ� ���, ��� �۾��� �� ��� �ð��� �����մϴ�.
// * @param sim �ùķ��̼� �ν��Ͻ�
// */
//static void accumulate_wait_ticks_if_realtime(Simulation* sim) {
//    ScenarioManager* sc = sim->scenario_manager;
//    if (sc->mode == MODE_REALTIME && sc->task_count > 0) {
//        TaskNode* c = sc->task_queue_head;
//        while (c) { sim->request_wait_ticks_sum++; c = c->next; }
//    }
//}
//
///**
// * @brief �ǽð� ����� ���, �ֱ������� ��ú��带 ������� �����ϰ� �ð��� �����ŵ�ϴ�.
// * @param sim �ùķ��̼� �ν��Ͻ�
// */
//static void maybe_report_realtime_dashboard(Simulation* sim) {
//    ScenarioManager* sc = sim->scenario_manager;
//    sc->time_step++;
//    if (sc->mode == MODE_REALTIME && (sc->time_step % DASHBOARD_INTERVAL_STEPS) == 0) {
//        simulation_report_realtime_dashboard(sim);
//    }
//}
//
//// APPEND_FMT ��ũ�ΰ� ���ǵǾ� �ִٴ� �����Ͽ� ��ü�� ����
//static int GridMap_renderToBuffer(char* buffer, size_t buffer_size,
//    const GridMap* map, const AgentManager* am)
//{
//    static char view[GRID_HEIGHT][GRID_WIDTH];
//    static const char* colors[GRID_HEIGHT][GRID_WIDTH];
//    char* p = buffer;
//    size_t rem = buffer_size;
//
//    // 1) �⺻ �ٴ� ä���
//    for (int y = 0; y < GRID_HEIGHT; y++) {
//        for (int x = 0; x < GRID_WIDTH; x++) {
//            const Node* n = &map->grid[y][x];
//            if (n->is_obstacle) { view[y][x] = '+'; colors[y][x] = C_WHT; }
//            else { view[y][x] = '.'; colors[y][x] = C_GRY; }
//        }
//    }
//
//    // 2) ������ ǥ�� (���� ���̸� ����, �ƴϸ� ���)
//    {
//        int ncs = map->num_charge_stations;
//        for (int i = 0; i < ncs; i++) {
//            Node* cs = map->charge_stations[i];
//            view[cs->y][cs->x] = 'e';
//            if (!g_renderer.simple_colors) {
//                int charging = FALSE;
//                for (int j = 0; j < MAX_AGENTS; j++) {
//                    if (am->agents[j].state == CHARGING && am->agents[j].pos == cs) { charging = TRUE; break; }
//                }
//                colors[cs->y][cs->x] = charging ? C_B_RED : C_B_YEL;
//            }
//        }
//    }
//
//    // 3) ��ǥ/������ ���� ǥ��
//    {
//        int ng = map->num_goals;
//        for (int i = 0; i < ng; i++) {
//            Node* g = map->goals[i];
//            if (g->is_parked) { view[g->y][g->x] = 'P'; if (!g_renderer.simple_colors) colors[g->y][g->x] = C_RED; }
//            else if (g->is_goal) { view[g->y][g->x] = 'G'; if (!g_renderer.simple_colors) colors[g->y][g->x] = C_GRN; }
//        }
//    }
//
//    // 4) ������Ʈ ��������
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        if (am->agents[i].pos) {
//            Node* n = am->agents[i].pos;
//            view[n->y][n->x] = am->agents[i].symbol;
//            colors[n->y][n->x] = AGENT_COLORS[i % 10];
//        }
//    }
//
//    // 5) ����� �׸��� ���(���ۿ��� ���)
//    //APPEND_FMT(p, rem, "%s\n--- D* Lite + WHCA* + WFG(SCC) + partial CBS ---%s\n", C_B_WHT, C_NRM);
//
//    if (g_renderer.simple_colors) {
//        for (int y = 0; y < GRID_HEIGHT; y++) {
//            for (int x = 0; x < GRID_WIDTH; x++) {
//                if (rem <= 1) break;
//                *p++ = view[y][x]; rem--;
//            }
//            if (rem <= 1) break;
//            *p++ = '\n'; rem--;
//        }
//    }
//    else {
//        for (int y = 0; y < GRID_HEIGHT; y++) {
//            for (int x = 0; x < GRID_WIDTH; x++) {
//                APPEND_FMT(p, rem, "%s%c%s", colors[y][x], view[y][x], C_NRM);
//            }
//            APPEND_FMT(p, rem, "\n");
//        }
//    }
//    APPEND_FMT(p, rem, "\n");
//
//    return (int)(p - buffer);
//}
//
//
///**
// * @brief �ùķ��̼��� ���� ����(���, ��, ������Ʈ ����, �α� ��)�� �����Ͽ�
// *        ���� ���÷��� ����(`g_display_buf`)�� �ؽ�Ʈ UI�� �����ϰ� ȭ�鿡 ����մϴ�.
// * @param sim �ùķ��̼� �ν��Ͻ�
// * @param is_paused ���� �ùķ��̼��� �Ͻ����� �������� ����
// */
//static void simulation_display_status(const Simulation* sim, int is_paused) {
//    const ScenarioManager* sc = sim->scenario_manager;
//    const AgentManager* am = sim->agent_manager;
//    const GridMap* map = sim->map;
//    const Logger* lg = sim->logger;
//    const int display_steps = (sim->total_executed_steps > 0) ? sim->total_executed_steps : sc->time_step;
//    const double avg_cpu_ms = (display_steps > 0) ? (sim->total_cpu_time_ms / (double)display_steps) : 0.0;
//
//    char* p = g_display_buf;
//    size_t rem = sizeof(g_display_buf);
//
//
//    // ����/��� ����
//    APPEND_FMT(p, rem, "%s", C_B_WHT);
//    if (sc->mode == MODE_CUSTOM) {
//        if (sc->current_phase_index < sc->num_phases) {
//            const DynamicPhase* ph = &sc->phases[sc->current_phase_index];
//            APPEND_FMT(p, rem, "--- Custom Scenario: %d/%d [Speed: %.1fx] ---  (Map #%d)",
//                sc->current_phase_index + 1, sc->num_phases, sc->speed_multiplier, sim->map_id);
//            if (is_paused) APPEND_FMT(p, rem, " %s[ PAUSED ]%s", C_B_YEL, C_B_WHT);
//            APPEND_FMT(p, rem, "\n");
//
//            APPEND_FMT(p, rem, "Time: %d, Current Task: %s (%d/%d)\n",
//                sc->time_step, ph->type_name, sc->tasks_completed_in_phase, ph->task_count);
//        }
//        else {
//            APPEND_FMT(p, rem, "--- Custom Scenario: All phases complete ---  (Map #%d)\n", sim->map_id);
//        }
//    }
//    else if (sc->mode == MODE_REALTIME) {
//        APPEND_FMT(p, rem, "--- Real-Time Simulation [Speed: %.1fx] ---  (Map #%d)",
//            sc->speed_multiplier, sim->map_id);
//        if (is_paused) APPEND_FMT(p, rem, " %s[ PAUSED ]%s", C_B_YEL, C_B_WHT);
//        APPEND_FMT(p, rem, "\n");
//
//        int park = 0, exitc = 0;
//        const TaskNode* t = sc->task_queue_head;
//        while (t) {
//            if (t->type == TASK_PARK) park++;
//            else if (t->type == TASK_EXIT) exitc++;
//            t = t->next;
//        }
//        APPEND_FMT(p, rem, "Time: %d / %d | Pending Tasks: %d (%sPark: %d%s, %sExit: %d%s)\n",
//            sc->time_step, REALTIME_MODE_TIMELIMIT, sc->task_count,
//            C_B_GRN, park, C_NRM, C_B_YEL, exitc, C_NRM);
//    }
//    APPEND_FMT(p, rem, "Parked Cars: %d/%d\n%s", am->total_cars_parked, map->num_goals, C_NRM);
//    APPEND_FMT(p, rem, "CPU Time (ms) - Last: %.3f | Avg: %.3f | Total: %.2f\n",
//        sim->last_step_cpu_time_ms, avg_cpu_ms, sim->total_cpu_time_ms);
//
//    // �˰��� ǥ��
//    {
//        const char* algo = "Default (WHCA*+D*Lite+WFG+CBS)";
//        if (sim->path_algo == PATHALGO_ASTAR_SIMPLE) algo = "A* (�ܼ�)";
//        else if (sim->path_algo == PATHALGO_DSTAR_BASIC) algo = "D* Lite (�⺻)";
//        APPEND_FMT(p, rem, "%sPath Algo:%s %s\n", C_B_WHT, C_NRM, algo);
//    }
//
//    APPEND_FMT(p, rem, "%sWHCA horizon:%s %d  | wf_edges(last): %d  | SCC(last): %d  | CBS(last): %s (exp:%d)\n",
//        C_B_WHT, C_NRM, g_whca_horizon, g_metrics.wf_edges_last, g_metrics.scc_last,
//        g_metrics.cbs_ok_last ? "OK" : "FAIL", g_metrics.cbs_exp_last);
//
//    // �� ������(���� �Լ��� �����ϰ� rem ���� �������� ���)
//    {
//        int w = GridMap_renderToBuffer(p, rem, map, am);
//        if (w < 0) w = 0;
//        if ((size_t)w >= rem) { p += rem - 1; rem = 1; }
//        else { p += w; rem -= w; }
//    }
//
//    // ������Ʈ ���� �г�
//    {
//        static const char* stS[] = {
//            "���","���� ��","���� ����(�� ��)","���� ��","���� ��","�����ҷ� �̵�","���� ��","���� ����(���� ��)"
//        };
//        static const char* stC[] = { C_GRY,C_YEL,C_CYN,C_YEL,C_GRN,C_B_RED,C_RED,C_CYN };
//
//        for (int i = 0; i < MAX_AGENTS; i++) {
//            const Agent* ag = &am->agents[i];
//            const char* c = AGENT_COLORS[i % 10];
//
//            char sbuf[100];
//            if (ag->state == CHARGING)
//                snprintf(sbuf, sizeof(sbuf), "���� ��... (%d)", ag->charge_timer);
//            else
//                snprintf(sbuf, sizeof(sbuf), "%s", stS[ag->state]);
//
//            APPEND_FMT(p, rem, "%sAgent %c%s: (%2d,%d) ",
//                c, ag->symbol, C_NRM,
//                ag->pos ? ag->pos->x : -1, ag->pos ? ag->pos->y : -1);
//
//            if (ag->goal)
//                APPEND_FMT(p, rem, "-> (%2d,%d) ", ag->goal->x, ag->goal->y);
//            else
//                APPEND_FMT(p, rem, "-> ����        ");
//
//            APPEND_FMT(p, rem, "[Mileage: %6.1f/%d] [%s%-*s%s]  [stuck:%d]\n",
//                ag->total_distance_traveled, (int)DISTANCE_BEFORE_CHARGE,
//                stC[ag->state], STATUS_STRING_WIDTH, sbuf, C_NRM, ag->stuck_steps);
//        }
//        APPEND_FMT(p, rem, "\n");
//    }
//
//    // �α�
//    APPEND_FMT(p, rem, "%s--- Simulation Log ---%s\n", C_B_WHT, C_NRM);
//    for (int i = 0; i < lg->log_count; i++) {
//        int idx = (lg->log_head + i) % LOG_BUFFER_LINES;
//        APPEND_FMT(p, rem, "%s%s%s\n", C_GRY, lg->logs[idx], C_NRM);
//        if (rem < 512) break; // ������ ���� �ݺ� ������ ���� ��ȣ
//    }
//
//    // *** NEW *** ���� Ű �ȳ� �߰�
//    ui_append_controls_help(&p, &rem);
//
//    // ȭ�� ���� (������ ��ŵ ����)
//    static int s_frame_counter = 0;
//    s_frame_counter++;
//    if ((s_frame_counter % (g_renderer.render_stride > 0 ? g_renderer.render_stride : 1)) == 0) {
//        ui_flush_display_buffer();
//    }
//}
//
//// =============================================================================
//// 4.5) Renderer Facade Implementation (delegates to simulation_display_status)
//// =============================================================================
//static void renderer_draw_frame_impl(struct Simulation_* sim, int is_paused) {
//    simulation_display_status(sim, is_paused);
//}
//static RendererFacade renderer_create_facade(void) {
//    RendererFacade f; f.vtbl.draw_frame = renderer_draw_frame_impl; return f;
//}
//
//
//// Logger
//Logger* logger_create() { Logger* l = (Logger*)calloc(1, sizeof(Logger)); if (!l) { perror("Logger"); exit(1); } return l; }
//void logger_destroy(Logger* l) { if (l) free(l); }
//void logger_log(Logger* l, const char* fmt, ...) {
//    va_list a; va_start(a, fmt);
//    int idx = (l->log_head + l->log_count) % LOG_BUFFER_LINES;
//    vsnprintf(l->logs[idx], LOG_BUFFER_WIDTH, fmt, a);
//    va_end(a);
//    if (l->log_count < LOG_BUFFER_LINES) l->log_count++;
//    else l->log_head = (l->log_head + 1) % LOG_BUFFER_LINES;
//}
//
//// OO-style aliases (kept as inline wrappers for readability; behavior unchanged)
//#define Logger_log logger_log
//#define Logger_create logger_create
//#define Logger_destroy logger_destroy
//
//// =============================================================================
//// ���� 5: �׸��� �� �� ���� (�� �ó����� 1~5 ����)
//// =============================================================================
///**
// * @brief ���� ��� ���� ��ֹ��� ���� �� �������� �ʱ�ȭ�մϴ�.
// * @param m �ʱ�ȭ�� GridMap ������
// */
//static void map_all_free(GridMap* m) {
//    grid_map_clear(m); // Initialize all cells to free space
//}
//
///**
// * @brief ���� �����ڸ��� ��(��ֹ�)�� �߰��մϴ�.
// * @param m ���� �߰��� GridMap ������
// */
//static void map_add_border_walls(GridMap* m) {
//    for (int x = 0; x < GRID_WIDTH; ++x) {
//        m->grid[0][x].is_obstacle = TRUE;
//        m->grid[GRID_HEIGHT - 1][x].is_obstacle = TRUE;
//    }
//    for (int y = 0; y < GRID_HEIGHT; ++y) {
//        m->grid[y][0].is_obstacle = TRUE;
//        m->grid[y][GRID_WIDTH - 1].is_obstacle = TRUE;
//    }
//}
//
///**
// * @brief ������ ��ǥ�� ���� ����(goal)�� ��ġ�մϴ�.
// *        �ش� ��ġ�� ��ȿ�ϰ�, ��ֹ��� �ƴϸ�, �̹� ���� ������ �ƴ� ��쿡�� ��ġ�˴ϴ�.
// * @param m ���� ������ ��ġ�� GridMap ������
// * @param x ��ġ�� x ��ǥ
// * @param y ��ġ�� y ��ǥ
// */
//static void map_place_goal(GridMap* m, int x, int y) {
//    if (!grid_is_valid_coord(x, y)) return;
//    Node* n = &m->grid[y][x];
//    if (!n->is_obstacle && !n->is_goal) {
//        n->is_goal = TRUE;
//        if (m->num_goals < MAX_GOALS) m->goals[m->num_goals++] = n;
//    }
//}
//
//static void map_place_charge(GridMap* m, int x, int y) {
//    if (!grid_is_valid_coord(x, y)) return;
//    Node* n = &m->grid[y][x];
//    if (!n->is_obstacle) {
//        if (m->num_charge_stations < MAX_CHARGE_STATIONS)
//            m->charge_stations[m->num_charge_stations++] = n;
//    }
//}
//
//static void map_place_agent_at(AgentManager* am, GridMap* m, int idx, int x, int y) {
//    if (idx < 0 || idx >= MAX_AGENTS) return;
//    Node* n = &m->grid[y][x];
//    am->agents[idx].pos = n;
//    am->agents[idx].home_base = n;
//    am->agents[idx].symbol = 'A' + idx; // A..J
//    am->agents[idx].heading = DIR_NONE;
//    am->agents[idx].rotation_wait = 0;
//}
//
//static void map_reserve_area_as_start(GridMap* m, int x0, int y0, int w, int h) {
//    for (int y = y0; y < y0 + h && y < GRID_HEIGHT; ++y)
//        for (int x = x0; x < x0 + w && x < GRID_WIDTH; ++x) {
//            Node* n = &m->grid[y][x];
//            n->is_obstacle = FALSE;
//            n->is_goal = FALSE;
//            n->is_temp = FALSE;
//        }
//}
//
//static void agent_manager_reset_for_new_map(AgentManager* am) {
//    if (!am) return;
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        if (am->agents[i].pf) { pathfinder_destroy(am->agents[i].pf); am->agents[i].pf = NULL; }
//        am->agents[i].id = i;
//        am->agents[i].symbol = 'A' + i;
//        am->agents[i].pos = NULL;
//        am->agents[i].home_base = NULL;
//        am->agents[i].goal = NULL;
//        am->agents[i].state = IDLE;
//        am->agents[i].total_distance_traveled = 0.0;
//        am->agents[i].charge_timer = 0;
//        am->agents[i].action_timer = 0;
//        am->agents[i].heading = DIR_NONE;
//        am->agents[i].rotation_wait = 0;
//        am->agents[i].stuck_steps = 0;
//        am->agents[i].metrics_task_active = 0;
//        am->agents[i].metrics_task_start_step = 0;
//        am->agents[i].metrics_distance_at_start = 0.0;
//        am->agents[i].metrics_turns_current = 0;
//    }
//    am->total_cars_parked = 0;
//}
//
//static void grid_map_clear(GridMap* map) {
//    memset(map, 0, sizeof(*map));
//    for (int y = 0; y < GRID_HEIGHT; ++y)
//        for (int x = 0; x < GRID_WIDTH; ++x) {
//            map->grid[y][x].x = x;
//            map->grid[y][x].y = y;
//            map->grid[y][x].is_obstacle = FALSE;
//            map->grid[y][x].is_goal = FALSE;
//            map->grid[y][x].is_temp = FALSE;
//            map->grid[y][x].is_parked = FALSE;
//            map->grid[y][x].reserved_by_agent = -1;
//        }
//    map->num_goals = 0;
//    map->num_charge_stations = 0;
//}
//
//static void grid_map_fill_from_string(GridMap* map, AgentManager* am, const char* m) {
//    grid_map_clear(map);
//
//    int x = 0, y = 0;
//    int last_was_cr = 0; /* CRLF�� �� ���� ���� ó���ϱ� ���� �÷��� */
//
//    for (const char* p = m; *p && y < GRID_HEIGHT; ++p) {
//        char ch = *p;
//
//        /* ���� ó��: CRLF/LF ��� ���� */
//        if (ch == '\r') {
//            x = 0; y++; last_was_cr = 1;
//            continue;
//        }
//        if (ch == '\n') {
//            if (!last_was_cr) { x = 0; y++; }
//            last_was_cr = 0;
//            continue;
//        }
//        last_was_cr = 0;
//
//        /* ���� �ٿ��� �׸��� �� �ʰ� ���ڴ� ������ ���� ���� ������ ��ŵ */
//        if (x >= GRID_WIDTH) continue;
//        if (y >= GRID_HEIGHT) break;
//
//        Node* n = &map->grid[y][x];
//
//        /* �⺻: ��ĭ(��ֹ� �ƴ�)�� ���� - grid_map_clear���� �̹� ������ */
//        n->is_obstacle = FALSE;
//        n->is_goal = FALSE;
//        n->is_temp = FALSE;
//        n->is_parked = FALSE;
//        n->reserved_by_agent = -1;
//
//        switch (ch) {
//        case '1':  /* ��/��ֹ� */
//            n->is_obstacle = TRUE;
//            break;
//
//        case 'A':  /* ������Ʈ A */
//            am->agents[0].pos = n;
//            am->agents[0].home_base = n;
//            break;
//
//        case 'B':  /* ������Ʈ B */
//            am->agents[1].pos = n;
//            am->agents[1].home_base = n;
//            break;
//
//        case 'C':  /* ������Ʈ C */
//            am->agents[2].pos = n;
//            am->agents[2].home_base = n;
//            break;
//
//        case 'D':  /* ������Ʈ D */
//            am->agents[3].pos = n;
//            am->agents[3].home_base = n;
//            break;
//
//        case 'G':  /* ���� ��ǥĭ */
//            n->is_goal = TRUE;
//            if (map->num_goals < MAX_GOALS) map->goals[map->num_goals++] = n;
//            break;
//
//        case 'e':  /* ������ */
//            if (map->num_charge_stations < MAX_CHARGE_STATIONS)
//                map->charge_stations[map->num_charge_stations++] = n;
//            break;
//
//        case '0':  /* ��ĭ */
//        default:
//            /* �ƹ� �͵� �� ��(��ĭ) */
//            break;
//        }
//
//        x++;
//    }
//
//    /* ���� ������ grid_map_clear()�� �⺻(��ĭ) ���� �� �»�� ���� ���·� �� */
//}
//
//
//void grid_map_load_scenario(GridMap* map, AgentManager* am, int scenario_id) {
//    agent_manager_reset_for_new_map(am);
//    grid_map_clear(map);
//
//    switch (scenario_id) {
//    case 1: {
//        static const char* MAP1 =
//            "1111111111111111111111111111111111111\n"
//            "D01GGG1GG1GGG1GGG1GGG1GGG1GGG1G11G111\n"
//            "A000000000000000000000000000000000001\n"
//            "B000000000000000000000000000000000001\n"
//            "C001GG1GG1GGG10001GGG1GGG1GGG1100e111\n"
//            "111111111111110001GGG1GGG1GGG11001111\n"
//            "100000000000000000000000000000000e111\n"
//            "100000000000000000000000000000000e111\n"
//            "11111111111111GGG1GGG1GGG1GGG1GG11111\n"
//            "1111111111111111111111111111111111111\n"
//            "1111111111111111111111111111111111111\n"
//            "1111111111111111111111111111111111111\n";
//        grid_map_fill_from_string(map, am, MAP1);
//        break;
//    }
//    case 2: map_build_hypermart(map, am);              break; // ������Ʈ ������
//    case 3: map_build_10agents_200slots(map, am);      break; // 8�� + 900ĭ + 16x6
//    case 4: map_build_biggrid_onegoal(map, am);        break; // ���ڵ��� + ������� 4��
//    case 5: map_build_cross_4agents(map, am);          break; // ���ڰ���
//    default:
//        map_build_hypermart(map, am); // fallback
//        break;
//    }
//}
//// #2: �˰��� �׽�Ʈ ���� �����۸�Ʈ �׽�Ʈ���� (���� ���޼� ���� ����)
//static void map_build_hypermart(GridMap* m, AgentManager* am) {
//    int x, y;
//
//    // 0) �ʱ�ȭ + �ܰ���
//    map_all_free(m);
//    map_add_border_walls(m);
//
//    // 1) ���θ� ��� ���Ƶΰ�(��ֹ�) �� ���θ� "���" ���
//    for (y = 1; y < GRID_HEIGHT - 1; ++y)
//        for (x = 1; x < GRID_WIDTH - 1; ++x) {
//            m->grid[y][x].is_obstacle = TRUE;
//            m->grid[y][x].is_goal = FALSE;
//        }
//
//    // 2) ��ŸƮ �е� + AGV 4��
//    map_reserve_area_as_start(m, 2, 2, 8, 5);
//    map_place_agent_at(am, m, 0, 2, 2);
//    map_place_agent_at(am, m, 1, 3, 2);
//    map_place_agent_at(am, m, 2, 4, 2);
//    map_place_agent_at(am, m, 3, 5, 2);
//
//    // 3) ��� �Ǵ�(���� 1����)
//    for (x = 1; x < GRID_WIDTH - 1; ++x) m->grid[6][x].is_obstacle = FALSE;
//
//    // 4) ���� ���� ����(���� 1����)
//    const int vCols[] = { 12, 22, 32, 42, 52, 62, 72 };
//    const int nV = (int)(sizeof(vCols) / sizeof(vCols[0]));
//    for (int i = 0; i < nV; ++i) {
//        int cx = vCols[i];
//        for (y = 1; y < GRID_HEIGHT - 1; ++y) m->grid[y][cx].is_obstacle = FALSE;
//    }
//
//    // 5) ���� ���� ������ 2�� + �߾� ����
//    for (x = 1; x < GRID_WIDTH - 1; ++x) {
//        m->grid[10][x].is_obstacle = FALSE;
//        m->grid[30][x].is_obstacle = FALSE;
//    }
//    for (y = 19; y <= 21; ++y)
//        for (x = 1; x < GRID_WIDTH - 1; ++x)
//            m->grid[y][x].is_obstacle = TRUE;
//
//    // ������� ���� ������ ����
//    for (int i = 0; i < nV; ++i) {
//        int cx = vCols[i];
//        for (y = 19; y <= 21; ++y) m->grid[y][cx].is_obstacle = FALSE;
//    }
//    // ���� �߰� ����
//    m->grid[20][34].is_obstacle = FALSE;
//    m->grid[20][50].is_obstacle = FALSE;
//
//    // 6) ���� ���� ����
//    for (y = 1; y < GRID_HEIGHT - 1; ++y) m->grid[y][4].is_obstacle = FALSE;
//    for (x = 4; x <= 10; ++x) m->grid[6][x].is_obstacle = FALSE;
//
//    // 7) Ǯ���� ����
//    const int pocketY[] = { 14, 16, 26, 28, 34 };
//    const int nP = (int)(sizeof(pocketY) / sizeof(pocketY[0]));
//    for (int i = 0; i < nV; ++i) {
//        int cx = vCols[i];
//        for (int k = 0; k < nP; ++k) {
//            int py = pocketY[k];
//            if (py >= 19 && py <= 21) continue;
//            if (py == 10 || py == 30) continue;
//            if (grid_is_valid_coord(cx - 1, py)) m->grid[py][cx - 1].is_obstacle = FALSE;
//            if (grid_is_valid_coord(cx + 1, py)) m->grid[py][cx + 1].is_obstacle = FALSE;
//        }
//    }
//
//    // 8) ������ 4��
//    map_place_charge(m, 12, 8);
//    map_place_charge(m, 42, 8);
//    map_place_charge(m, 42, 32);
//    map_place_charge(m, 72, 8);
//
//    // 9) ���� ���� �� ��ũ
//    int markRow[GRID_HEIGHT] = { 0 };
//    markRow[6] = 1;
//    markRow[10] = 1;
//    markRow[19] = 1; markRow[20] = 1; markRow[21] = 1;
//    markRow[30] = 1;
//
//    // 10) �� ���� ���� �翷 �������� ��ǥĭ
//    const int y_min = 8, y_max = GRID_HEIGHT - 4;
//    for (int i = 0; i < nV; ++i) {
//        int roadX = vCols[i];
//        int leftCol = roadX - 1;
//        int rightCol = roadX + 1;
//        for (y = y_min; y <= y_max; ++y) {
//            if (markRow[y]) continue;
//            if (grid_is_valid_coord(leftCol, y)) { m->grid[y][leftCol].is_obstacle = FALSE; map_place_goal(m, leftCol, y); }
//            if (grid_is_valid_coord(rightCol, y)) { m->grid[y][rightCol].is_obstacle = FALSE; map_place_goal(m, rightCol, y); }
//        }
//    }
//
//    // 11) (�ڼ��� �ٽ�) ���� �ܰ� ����: ���� + ��ǥ �� ������ ��ġ
//    const int side_right_col = GRID_WIDTH - 4; // ��ǥ ��
//    const int side_right_road = GRID_WIDTH - 5; // �� ���� '����'�� ���� ��
//    // 11-1) �������� ���� (���� ���� ����)
//    for (y = 1; y < GRID_HEIGHT - 1; ++y) m->grid[y][side_right_road].is_obstacle = FALSE;
//    for (y = 19; y <= 21; ++y)            m->grid[y][side_right_road].is_obstacle = FALSE; // ���� ���� ����
//    // 11-2) ���� �����ʿ� ��ǥĭ(�׻� ������� Ȯ��)
//    for (y = y_min; y <= y_max; ++y) {
//        if (markRow[y]) continue;
//        if (grid_is_valid_coord(side_right_col, y)) {
//            m->grid[y][side_right_col].is_obstacle = FALSE;
//            map_place_goal(m, side_right_col, y);
//        }
//    }
//
//    // 12) ��ŸƮ/���� ��ó ��ǥ ����
//    for (y = 2; y <= 8; ++y)
//        for (x = 2; x <= 8; ++x)
//            m->grid[y][x].is_goal = FALSE;
//    for (y = 1; y < GRID_HEIGHT - 1; ++y) m->grid[y][4].is_goal = FALSE;
//
//    // 13) (������ġ) ��ǥ ����: 4���� ��� ��/�ܰ��̸� ��ǥ ����
//    m->num_goals = 0;
//    for (y = 1; y < GRID_HEIGHT - 1; ++y) {
//        for (x = 1; x < GRID_WIDTH - 1; ++x) {
//            Node* n = &m->grid[y][x];
//            if (!n->is_goal) continue;
//            int ok = 0;
//            const int dx[4] = { 1,-1,0,0 }, dy[4] = { 0,0,1,-1 };
//            for (int k = 0; k < 4; k++) {
//                int nx = x + dx[k], ny = y + dy[k];
//                if (!grid_is_valid_coord(nx, ny)) continue;
//                if (!m->grid[ny][nx].is_obstacle) { ok = 1; break; }
//            }
//            if (!ok) n->is_goal = FALSE; // �� ��ǥ ����
//            if (n->is_goal && m->num_goals < MAX_GOALS) m->goals[m->num_goals++] = n;
//        }
//    }
//}
//
//
//
//
//
//
//
//// (Map #3 ���ŵ�)
//
//
//
//// #3: 8 agents + 900 parking slots
//// - ���� 2���� ���ε���(x=2,3) + y=6,7 ���� 2���� �Ǵ� ����
//// - ��ŸƮ ������ 10x4 �� 16x6���� '��¦' Ȯ��
//// - Ȯ��� ��ŸƮ ���� ���ο� ������ 4��(2x2) ��ġ
//// - ��� ����ĭ�� ���ο� �� ���� ���ϵ��� ��ġ�ϸ� ���� 2���� �ֺ����� G�� ������ ����
//static void map_build_10agents_200slots(GridMap* m, AgentManager* am) {
//    int x, y;
//
//    /* 0) �ʱ�ȭ + �ܰ��� */
//    map_all_free(m);
//    map_add_border_walls(m);
//
//    /* 1) ���θ� ���� ��ֹ��� ä��(���θ� ��´�) */
//    for (y = 1; y < GRID_HEIGHT - 1; ++y)
//        for (x = 1; x < GRID_WIDTH - 1; ++x) {
//            m->grid[y][x].is_obstacle = TRUE;
//            m->grid[y][x].is_goal = FALSE;
//        }
//
//    /* 2) ��ŸƮ ���� Ȯ��: (2,2)���� 16��6 */
//    const int sx0 = 2, sy0 = 2;
//    const int sW = 16, sH = 6;                 // �� Ȯ��
//    map_reserve_area_as_start(m, sx0, sy0, sW, sH);
//
//    /* A..H ��ġ(8��) */
//    for (int i = 0; i < 8; ++i) {
//        int row = i / 5, col = i % 5;
//        map_place_agent_at(am, m, i, sx0 + col * 2, sy0 + row * 2);
//    }
//
//    /* ===== ���� �Ķ���� ===== */
//    const int lane_w = 1;          // ������ ���� ��(1����)
//    const int y_min = 8;          // ���� ���� y (��ŸƮ/�Ǵ� �Ʒ�)
//    const int y_max = GRID_HEIGHT - 5;
//
//    // ���� ���ε���(1����): x=16..(W-6), step 4
//    const int ax_start = 16;
//    const int ax_end = GRID_WIDTH - 6;
//    const int ax_step = 4;
//
//    // ���� ���ᵵ��(1����): y=10..(H-6), step 6
//    const int cross_start = 10;
//    const int cross_end = GRID_HEIGHT - 6;
//    const int cross_step = 6;
//
//    /* 3) ���� 2���� ���� �ֵ���(x=2,3) */
//    for (y = 1; y < GRID_HEIGHT - 1; ++y) {
//        if (grid_is_valid_coord(2, y)) m->grid[y][2].is_obstacle = FALSE;
//        if (grid_is_valid_coord(3, y)) m->grid[y][3].is_obstacle = FALSE;
//    }
//
//    /* 4) ��ŸƮ �Ʒ� ���� 2���� �Ǵ�(y=6,7) */
//    for (x = 1; x < GRID_WIDTH - 1; ++x) {
//        if (grid_is_valid_coord(x, 6)) m->grid[6][x].is_obstacle = FALSE;
//        if (grid_is_valid_coord(x, 7)) m->grid[7][x].is_obstacle = FALSE;
//    }
//
//    /* 5) ���� ���ε���(1����) */
//    for (x = ax_start; x <= ax_end; x += ax_step)
//        for (y = 1; y < GRID_HEIGHT - 1; ++y)
//            m->grid[y][x].is_obstacle = FALSE;
//
//    /* 6) ���� ���ᵵ��(1����) */
//    for (y = cross_start; y <= cross_end; y += cross_step)
//        for (x = 1; x < GRID_WIDTH - 1; ++x)
//            m->grid[y][x].is_obstacle = FALSE;
//
//    /* 7) �������� 4��: ���� �ڽ� ������ �������� ��ġ(2��2 ����) */
//    // ���� ������ ���� ���(sx0) ��ó�� ���� ��ġ�Ͽ� ���ټ� Ȯ��
//    {
//        int cxL = sx0;          // ���� �ڽ� ������ ��
//        int cyT = sy0 + 1;      // ���� �ڽ� ��ܿ��� �� ĭ �Ʒ�
//        map_place_charge(m, cxL, cyT);
//        map_place_charge(m, cxL + 1, cyT);
//        map_place_charge(m, cxL, cyT + 2);
//        map_place_charge(m, cxL + 1, cyT + 2);
//    }
//
//    /* 8) ���� ���� ��/�� ��ũ (������/�Ǵ� �� + ��� �������� ��) */
//    int markRow[GRID_HEIGHT] = { 0 };
//    markRow[6] = 1; markRow[7] = 1;                  // 2���� �Ǵ�
//    for (y = cross_start; y <= cross_end; y += cross_step)
//        if (y >= 0 && y < GRID_HEIGHT) markRow[y] = 1;
//
//    int markCol[GRID_WIDTH] = { 0 };
//    markCol[2] = 1; markCol[3] = 1;                  // ���� 2���� �ֵ���
//    for (x = ax_start; x <= ax_end; x += ax_step)
//        if (x >= 0 && x < GRID_WIDTH) markCol[x] = 1;
//
//    /* 9) 1�� �н�: ���� ���ε��� ��/�� �������� ���� ��ġ */
//    const int target = 900;
//    int placed = 0;
//    for (x = ax_start; x <= ax_end && placed < target; x += ax_step) {
//        int leftCol = x - 1;
//        int rightCol = x + lane_w; // x+1
//        for (y = y_min; y <= y_max && placed < target; ++y) {
//            if (markRow[y]) continue;               // ������/�Ǵ� ���� ���
//
//            if (grid_is_valid_coord(leftCol, y) && placed < target) {
//                m->grid[y][leftCol].is_obstacle = FALSE;
//                map_place_goal(m, leftCol, y);
//                placed++;
//            }
//            if (grid_is_valid_coord(rightCol, y) && placed < target) {
//                m->grid[y][rightCol].is_obstacle = FALSE;
//                map_place_goal(m, rightCol, y);
//                placed++;
//            }
//        }
//    }
//
//    /* 10) 2�� �н�(������): ���ε��� ��/�Ʒ� �� ĭ(�������� ���� �ǳʶ�) */
//    if (placed < target) {
//        for (y = cross_start; y <= cross_end && placed < target; y += cross_step) {
//            int row = y;
//            for (x = 2; x < GRID_WIDTH - 2 && placed < target; ++x) {
//                if (markCol[x]) continue;                   // ���� ��� ����
//                if (m->grid[row][x].is_obstacle != FALSE) continue; // ���� ���ε��θ�
//
//                if (grid_is_valid_coord(x, row - 1) &&
//                    !m->grid[row - 1][x].is_goal && placed < target) {
//                    m->grid[row - 1][x].is_obstacle = FALSE;
//                    map_place_goal(m, x, row - 1);
//                    placed++;
//                }
//                if (grid_is_valid_coord(x, row + 1) &&
//                    !m->grid[row + 1][x].is_goal && placed < target) {
//                    m->grid[row + 1][x].is_obstacle = FALSE;
//                    map_place_goal(m, x, row + 1);
//                    placed++;
//                }
//            }
//        }
//    }
//
//    /* 11) ����: ���� �ֵ���(x=2,3)�� ���� G ���� */
//    for (y = 1; y < GRID_HEIGHT - 1; ++y) {
//        m->grid[y][2].is_goal = FALSE;
//        m->grid[y][3].is_goal = FALSE;
//    }
//}
//
//
//
//
//
//
//// ����������������������������������������������������������������������������������������������������������������������������
//// 1-���� ������� + 1-ĭ ������ + ���ڵ��η��� ����(����� ���ڼ��� ����)
//// cx,cy : ��� �߽� / Wg,Hg : ��� ����x���� ũ��(��: 6x2)
//// vstep,hstep,vx0,hy0 : ���ڵ��� ����/������ǥ (����/����)
//// CX,CY : �� �߽�(���ۿ��� ����, ���� ���� ���ÿ� ���)
//// ����������������������������������������������������������������������������������������������������������������������������
//static void carve_block_1lane(GridMap* m,
//    int cx, int cy, int Wg, int Hg,
//    int vstep, int hstep, int vx0, int hy0,
//    int CX, int CY)
//{
//    int x, y;
//
//    // 1) ��ǥ ���(���� ���� + G ǥ��)
//    int gx0 = cx - (Wg / 2 - 1);
//    int gx1 = gx0 + Wg - 1;
//    int gy0 = cy - (Hg / 2);
//    int gy1 = gy0 + Hg - 1;
//
//    for (y = gy0; y <= gy1; ++y)
//        for (x = gx0; x <= gx1; ++x) {
//            if (!grid_is_valid_coord(x, y)) continue;
//            m->grid[y][x].is_obstacle = FALSE;
//            map_place_goal(m, x, y);
//        }
//
//    // 2) ����� ���δ� 1ĭ �� ������
//    int rxL = gx0 - 1, rxR = gx1 + 1;
//    int ryT = gy0 - 1, ryB = gy1 + 1;
//
//    for (x = gx0 - 1; x <= gx1 + 1; ++x) {
//        if (grid_is_valid_coord(x, ryT)) m->grid[ryT][x].is_obstacle = FALSE;
//        if (grid_is_valid_coord(x, ryB)) m->grid[ryB][x].is_obstacle = FALSE;
//    }
//    for (y = gy0 - 1; y <= gy1 + 1; ++y) {
//        if (grid_is_valid_coord(rxL, y)) m->grid[y][rxL].is_obstacle = FALSE;
//        if (grid_is_valid_coord(rxR, y)) m->grid[y][rxR].is_obstacle = FALSE;
//    }
//
//    // 3) ���ڵ��ο� 1ĭ ������ ����(����� ����/���� ���ڼ��� ����)
//    //    ���� ���ڼ� x = vx0 + k*vstep, ���� ���ڼ� y = hy0 + k*hstep
//    int kx = (cx - vx0 + vstep / 2) / vstep;
//    int vx = vx0 + kx * vstep;
//    if (vx < 1) vx = 1;
//    if (vx > GRID_WIDTH - 2) vx = GRID_WIDTH - 2;
//
//    int ky = (cy - hy0 + hstep / 2) / hstep;
//    int hy = hy0 + ky * hstep;
//    if (hy < 1) hy = 1;
//    if (hy > GRID_HEIGHT - 2) hy = GRID_HEIGHT - 2;
//
//    // ���� ����: �� ��/�� �� �߽�(CY)�� �� ����� �� ����
//    {
//        int linkY = (abs(ryT - CY) <= abs(ryB - CY)) ? ryT : ryB;
//        if (vx <= rxL) { for (x = vx; x <= rxL; ++x) m->grid[linkY][x].is_obstacle = FALSE; }
//        else if (vx >= rxR) { for (x = rxR; x <= vx; ++x) m->grid[linkY][x].is_obstacle = FALSE; }
//        else { m->grid[linkY][vx].is_obstacle = FALSE; }
//    }
//    // ���� ����: �� ��/�� �� �߽�(CX)�� �� ����� �� ����
//    {
//        int linkX = (abs(rxL - CX) <= abs(rxR - CX)) ? rxL : rxR;
//        if (hy <= ryT) { for (y = hy; y <= ryT; ++y) m->grid[y][linkX].is_obstacle = FALSE; }
//        else if (hy >= ryB) { for (y = ryB; y <= hy; ++y) m->grid[y][linkX].is_obstacle = FALSE; }
//        else { m->grid[hy][linkX].is_obstacle = FALSE; }
//    }
//}
//
//// #4: �߾� ����(10x4) + �»�/���/����/���� 4���
////     1ĭ ���ڵ���(vstep=5,hstep=5) + y=6 �Ǵ� + ��� ���� 1����
//static void map_build_biggrid_onegoal(GridMap* m, AgentManager* am) {
//    map_all_free(m);
//    map_add_border_walls(m);
//
//    int x, y;
//
//    // ���� ���� ����
//    for (y = 1; y < GRID_HEIGHT - 1; ++y)
//        for (x = 1; x < GRID_WIDTH - 1; ++x) {
//            m->grid[y][x].is_obstacle = TRUE;
//            m->grid[y][x].is_goal = FALSE;
//        }
//
//    // ���� ���ۿ���: �� ���߾� 10x4 (A..J) ����
//    const int CX = GRID_WIDTH / 2;     // 82x42 ����: 41
//    const int CY = GRID_HEIGHT / 2;    // 21
//    const int sW = 10, sH = 4;
//    const int sx0 = CX - sW / 2;
//    const int sy0 = CY - sH / 2;
//    map_reserve_area_as_start(m, sx0, sy0, sW, sH);
//    for (int i = 0; i < 10; ++i) {
//        int row = i / 5, col = i % 5;
//        map_place_agent_at(am, m, i, sx0 + col * 2, sy0 + row * 2);
//    }
//
//    // ���� 1ĭ ���ڵ��� ����
//    const int vstep = 5, hstep = 5;
//    const int vx0 = 6;   // ���� ���� ���� x
//    const int hy0 = 9;   // ���� ���� ���� y
//
//    // ���� ���ڼ�
//    for (x = vx0; x < GRID_WIDTH - 1; x += vstep)
//        for (y = 1; y < GRID_HEIGHT - 1; ++y)
//            m->grid[y][x].is_obstacle = FALSE;
//
//    // ���� ���ڼ�
//    for (y = hy0; y < GRID_HEIGHT - 1; y += hstep)
//        for (x = 1; x < GRID_WIDTH - 1; ++x)
//            m->grid[y][x].is_obstacle = FALSE;
//
//    // ��ŸƮ �� ���� �Ǵ�(y=6, 1ĭ)
//    for (x = 1; x < GRID_WIDTH - 1; ++x)
//        m->grid[6][x].is_obstacle = FALSE;
//
//    // ���� 4���� �������(���� ��ġ ����, �߾ӿ��� ����� ����������) ����
//    const int Wg = 6, Hg = 2;
//    const int bxL = 8;
//    const int bxR = GRID_WIDTH - 8;
//    const int byT = 8;
//    const int byB = GRID_HEIGHT - 8;
//
//    // �»��
//    carve_block_1lane(m, bxL, byT, Wg, Hg, vstep, hstep, vx0, hy0, CX, CY);
//    // ����
//    carve_block_1lane(m, bxR, byT, Wg, Hg, vstep, hstep, vx0, hy0, CX, CY);
//    // ���ϴ�
//    carve_block_1lane(m, bxL, byB, Wg, Hg, vstep, hstep, vx0, hy0, CX, CY);
//    // ���ϴ�
//    carve_block_1lane(m, bxR, byB, Wg, Hg, vstep, hstep, vx0, hy0, CX, CY);
//
//    // ������ 4��(�߾� �ֺ�)
//    if (grid_is_valid_coord(CX, CY - 6)) map_place_charge(m, CX, CY - 6);
//    if (grid_is_valid_coord(CX, CY + 6)) map_place_charge(m, CX, CY + 6);
//    if (grid_is_valid_coord(CX - 6, CY)) map_place_charge(m, CX - 6, CY);
//    if (grid_is_valid_coord(CX + 6, CY)) map_place_charge(m, CX + 6, CY);
//
//    // goals[] �籸��
//    m->num_goals = 0;
//    for (y = 1; y < GRID_HEIGHT - 1; ++y)
//        for (x = 1; x < GRID_WIDTH - 1; ++x)
//            if (m->grid[y][x].is_goal && m->num_goals < MAX_GOALS)
//                m->goals[m->num_goals++] = &m->grid[y][x];
//}
//
//
//
//
//
//
//
//
//
//
//
//// #5: ���ڰ� �� (Cross) - �߾� ������, �� �� ���� ������Ʈ, �� ������Ʈ���� 4ĭ ���� ������ ����ĭ
//static void map_build_cross_4agents(GridMap* m, AgentManager* am) {
//    int x, y;
//
//    // 0) �ʱ�ȭ + �ܰ���
//    map_all_free(m);
//    map_add_border_walls(m);
//
//    // 1) ���θ� ��ֹ��� ä��� �� ���ڰ�(����/���� 1����)�� ����
//    for (y = 1; y < GRID_HEIGHT - 1; ++y)
//        for (x = 1; x < GRID_WIDTH - 1; ++x) {
//            m->grid[y][x].is_obstacle = TRUE;
//            m->grid[y][x].is_goal = FALSE;
//        }
//
//    const int CX = GRID_WIDTH / 2;
//    const int CY = GRID_HEIGHT / 2;
//
//    for (y = 1; y < GRID_HEIGHT - 1; ++y) m->grid[y][CX].is_obstacle = FALSE; // ���� ��
//    for (x = 1; x < GRID_WIDTH - 1; ++x) m->grid[CY][x].is_obstacle = FALSE; // ���� ��
//
//    // 2) ������Ʈ ��ġ: ��/��/��/�� �� ��(�ܰ��� �ٷ� ����)
//    map_place_agent_at(am, m, 0, 1, CY);                    // ���� ��
//    map_place_agent_at(am, m, 1, GRID_WIDTH - 2, CY);       // ���� ��
//    map_place_agent_at(am, m, 2, CX, 1);                    // ���� ��
//    map_place_agent_at(am, m, 3, CX, GRID_HEIGHT - 2);      // ���� ��
//
//    // 3) �� ������Ʈ ��ġ���� 4ĭ ���� ������ ���� ��ǥĭ ��ġ
//    map_place_goal(m, 1 + 4, CY);                           // ���� �� �������� 4ĭ
//    map_place_goal(m, GRID_WIDTH - 2 - 4, CY);              // ���� �� �������� 4ĭ
//    map_place_goal(m, CX, 1 + 4);                           // ���� �� �������� 4ĭ
//    map_place_goal(m, CX, GRID_HEIGHT - 2 - 4);             // ���� �� �������� 4ĭ
//
//    // 4) �߾� ������ 1��
//    map_place_charge(m, CX, CY);
//}
//GridMap* grid_map_create(AgentManager* am) {
//    GridMap* m = (GridMap*)calloc(1, sizeof(GridMap)); if (!m) { perror("GridMap"); exit(1); }
//    grid_map_load_scenario(m, am, 1); // �⺻ �� = 1
//    return m;
//}
//void grid_map_destroy(GridMap* m) { if (m) free(m); }
//
//int grid_is_valid_coord(int x, int y) { return (x >= 0 && x < GRID_WIDTH&& y >= 0 && y < GRID_HEIGHT); }
//
//int grid_is_node_blocked(const GridMap* map, const AgentManager* am, const Node* n, const struct Agent_* agent) {
//    if (n->is_obstacle || n->is_parked || n->is_temp) return TRUE;
//
//    // ���� ���� ����(RETURNING_HOME_EMPTY)�� �� ���� ������ �̿��� �� ����
//    if (agent && agent->state == RETURNING_HOME_EMPTY && n->is_goal && !n->is_parked) {
//        return TRUE;
//    }
//
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        if (am->agents[i].pos == n && am->agents[i].state == CHARGING) return TRUE;
//    }
//    return FALSE;
//}
//
//// =============================================================================
//// ���� 6: �ó����� �� �۾� ����
//// =============================================================================
///**
// * @brief �ǽð� ��� �۾� ��⿭�� ��� �����ϰ� ī���͸� �ʱ�ȭ�մϴ�.
// * @param s �ó����� �Ŵ��� ������
// */
//static void scenario_manager_clear_task_queue(ScenarioManager* s) {
//    TaskNode* c = s->task_queue_head; while (c) { TaskNode* nx = c->next; free(c); c = nx; }
//    s->task_queue_head = s->task_queue_tail = NULL; s->task_count = 0;
//}
///**
// * @brief �ó����� �Ŵ����� �����մϴ�.
// *        �⺻ �ӵ�, Ȯ��, �ð� ���� �ʵ带 �ʱ�ȭ�մϴ�.
// * @return ������ ScenarioManager ������ (���� �� ����)
// */
//ScenarioManager* scenario_manager_create() {
//    ScenarioManager* s = (ScenarioManager*)calloc(1, sizeof(ScenarioManager));
//    if (!s) { perror("Scenario"); exit(1); }
//    s->simulation_speed = 100; s->speed_multiplier = 1.0f;
//    s->park_chance = 40; s->exit_chance = 30;
//    return s;
//}
///**
// * @brief �ó����� �Ŵ����� �ı��ϰ� ���� ��⿭�� �����մϴ�.
// * @param s �ı��� ScenarioManager ������
// */
//void scenario_manager_destroy(ScenarioManager* s) { if (s) { scenario_manager_clear_task_queue(s); free(s); } }
//
//// --- AgentOps VTable: ������Ʈ �۾� ����/��ǥ ���� �߻�ȭ ---
//typedef struct AgentOps_ {
//    void (*beginTaskPark)(Agent* ag, ScenarioManager* sc, Logger* lg);
//    void (*beginTaskExit)(Agent* ag, ScenarioManager* sc, Logger* lg);
//    void (*setGoalIfNeeded)(Agent* ag, GridMap* map, AgentManager* am, Logger* lg);
//} AgentOps;
//
///**
// * @brief ������Ʈ�� ����(PARK) �۾��� ���۽�ŵ�ϴ�.
// *        �۾� ��Ʈ���� �ʱ�ȭ�ϰ� �α׸� ����մϴ�.
// * @param ag ��� ������Ʈ
// * @param sc �ó����� �Ŵ���
// * @param lg �ΰ�
// */
//static void AgentOps_beginTaskPark_impl(Agent* ag, ScenarioManager* sc, Logger* lg) { agent_begin_task_park(ag, sc, lg); }
///**
// * @brief ������Ʈ�� ����(EXIT) �۾��� ���۽�ŵ�ϴ�.
// *        �۾� ��Ʈ���� �ʱ�ȭ�ϰ� �α׸� ����մϴ�.
// * @param ag ��� ������Ʈ
// * @param sc �ó����� �Ŵ���
// * @param lg �ΰ�
// */
//static void AgentOps_beginTaskExit_impl(Agent* ag, ScenarioManager* sc, Logger* lg) { agent_begin_task_exit(ag, sc, lg); }
//// forward declaration to avoid implicit declaration when called here
//static void agent_set_goal(Agent* ag, GridMap* map, AgentManager* am, Logger* lg);
///**
// * @brief ��ǥ�� ���� �̵� ���� ������Ʈ���� ���¿� �´� ��ǥ�� �����մϴ�.
// *        IDLE/CHARGING ���´� �����մϴ�.
// * @param ag ��� ������Ʈ
// * @param map �׸��� ��
// * @param am ������Ʈ �Ŵ���
// * @param lg �ΰ�
// */
//static void AgentOps_setGoalIfNeeded_impl(Agent* ag, GridMap* map, AgentManager* am, Logger* lg) {
//    if (ag->goal == NULL && ag->state != IDLE && ag->state != CHARGING) agent_set_goal(ag, map, am, lg);
//}
//static AgentOps g_agent_ops = { AgentOps_beginTaskPark_impl, AgentOps_beginTaskExit_impl, AgentOps_setGoalIfNeeded_impl };
///**
// * @brief �ǽð� ��� �۾� ��⿭�� ���Ͽ� �۾��� �߰��մϴ�.
// *        �ִ� �۾� ���� �����ϸ� ���õ˴ϴ�.
// * @param s �ó����� �Ŵ���
// * @param t �۾� Ÿ��(TASK_PARK/TASK_EXIT)
// */
//static void add_task_to_queue(ScenarioManager* s, TaskType t) {
//    if (s->task_count >= MAX_TASKS) return;
//    TaskNode* n = (TaskNode*)malloc(sizeof(TaskNode)); if (!n) { perror("task"); return; }
//    n->type = t; n->created_at_step = s->time_step; n->next = NULL;
//    if (!s->task_queue_head) { s->task_queue_head = s->task_queue_tail = n; }
//    else { s->task_queue_tail->next = n; s->task_queue_tail = n; }
//    s->task_count++;
//}
//
//// =============================================================================
//// ���� 7: �켱���� ť (Pathfinder�� OPEN ����Ʈ)
//// =============================================================================
//typedef struct { double k1; double k2; } _cmpKey; // same as Key
//
///**
// * @brief D* Lite�� Key �� �Լ��Դϴ�.
// * @param a ���� Key
// * @param b ���� Key
// * @return a<b:-1, a>b:1, ������ 0
// */
//static int compare_keys(Key a, Key b) {
//    if (a.k1 < b.k1 - 1e-9) return -1;
//    if (a.k1 > b.k1 + 1e-9) return  1;
//    if (a.k2 < b.k2 - 1e-9) return -1;
//    if (a.k2 > b.k2 + 1e-9) return  1;
//    return 0;
//}
///**
// * @brief Pathfinder ���� ��(SearchCell)�� ���� �����͸� ��ȯ�մϴ�.
// * @param pf Pathfinder
// * @param n ��� ���
// * @return SearchCell ������
// */
//static inline SearchCell* cell_of(Pathfinder* pf, const Node* n) { return &pf->cells[n->y][n->x]; }
///**
// * @brief ����� ���� Key ���� ��ȯ�մϴ�.
// * @param pf Pathfinder
// * @param n ��� ���
// * @return Key ��
// */
//static inline Key key_of(Pathfinder* pf, const Node* n) { return cell_of(pf, n)->key; }
//
///**
// * @brief ��� �켱���� ť(��)�� �ʱ�ȭ�մϴ�.
// * @param pq ��� ť
// * @param cap �ִ� �뷮
// */
//static void pq_init(NodePQ* pq, int cap) {
//    pq->nodes = (Node**)malloc(sizeof(Node*) * cap); pq->size = 0; pq->capacity = cap;
//}
///**
// * @brief ��� �켱���� ť ���ҽ��� �����մϴ�.
// * @param pq ��� ť
// */
//static void pq_free(NodePQ* pq) { if (pq && pq->nodes) { free(pq->nodes); pq->nodes = NULL; } }
//
///**
// * @brief �� ���� �� ��带 ��ȯ�ϰ� �ε��� ������ �����մϴ�.
// * @param pf Pathfinder (�ε��� ���ſ�)
// * @param a ��� ������ ���� 1
// * @param b ��� ������ ���� 2
// */
//static void pq_swap(Pathfinder* pf, Node** a, Node** b) {
//    Node* t = *a; *a = *b; *b = t;
//    {
//        int ia = cell_of(pf, *a)->pq_index;
//        int ib = cell_of(pf, *b)->pq_index;
//        cell_of(pf, *a)->pq_index = ib;
//        cell_of(pf, *b)->pq_index = ia;
//    }
//}
///**
// * @brief �� ���� �ø���(���� �� ������).
// * @param pf Pathfinder
// * @param pq ��� ť
// * @param i ������ ���� �ε���
// */
//static void heapify_up(Pathfinder* pf, NodePQ* pq, int i) {
//    if (i == 0) return;
//    int p = (i - 1) / 2;
//    if (compare_keys(key_of(pf, pq->nodes[i]), key_of(pf, pq->nodes[p])) < 0) {
//        pq_swap(pf, &pq->nodes[i], &pq->nodes[p]);
//        heapify_up(pf, pq, p);
//    }
//}
///**
// * @brief �� �Ʒ��� ������(��Ʈ ��ü �� ������).
// * @param pf Pathfinder
// * @param pq ��� ť
// * @param i ������ ���� �ε���
// */
//static void heapify_down(Pathfinder* pf, NodePQ* pq, int i) {
//    int l = 2 * i + 1, r = 2 * i + 2, s = i;
//    if (l < pq->size && compare_keys(key_of(pf, pq->nodes[l]), key_of(pf, pq->nodes[s])) < 0) s = l;
//    if (r < pq->size && compare_keys(key_of(pf, pq->nodes[r]), key_of(pf, pq->nodes[s])) < 0) s = r;
//    if (s != i) { pq_swap(pf, &pq->nodes[i], &pq->nodes[s]); heapify_down(pf, pq, s); }
//}
///**
// * @brief Ư�� ��尡 ť�� ���ԵǾ� �ִ��� Ȯ���մϴ�.
// * @param pf Pathfinder
// * @param n ���
// * @return ����:1, �ƴϸ� 0
// */
//static int pq_contains(Pathfinder* pf, const Node* n) { return cell_of(pf, n)->in_pq; }
///**
// * @brief ť�� �ֻ�� Key�� ��ȯ�մϴ�. ��������� (INF,INF).
// * @param pf Pathfinder
// * @param pq ��� ť
// * @return �ֻ�� Key
// */
//static Key pq_top_key(Pathfinder* pf, const NodePQ* pq) {
//    if (pq->size == 0) return make_key(INF, INF);
//    return key_of(pf, pq->nodes[0]);
//}
///**
// * @brief ��带 ť�� �����մϴ�.
// * @param pf Pathfinder
// * @param pq ��� ť
// * @param n ������ ���
// */
//static void pq_push(Pathfinder* pf, NodePQ* pq, Node* n) {
//    if (pq->size >= pq->capacity) return;
//    SearchCell* c = cell_of(pf, n);
//    c->in_pq = TRUE; c->pq_index = pq->size; pq->nodes[pq->size++] = n;
//    heapify_up(pf, pq, pq->size - 1);
//}
///**
// * @brief ť�� �ֻ�� ��带 �����ϰ� ��ȯ�մϴ�.
// * @param pf Pathfinder
// * @param pq ��� ť
// * @return �˵� ��� ������ (������ NULL)
// */
//static Node* pq_pop(Pathfinder* pf, NodePQ* pq) {
//    if (pq->size == 0) return NULL;
//    Node* top = pq->nodes[0];
//    SearchCell* ct = cell_of(pf, top); ct->in_pq = FALSE; ct->pq_index = -1;
//    pq->size--;
//    if (pq->size > 0) {
//        pq->nodes[0] = pq->nodes[pq->size];
//        cell_of(pf, pq->nodes[0])->pq_index = 0;
//        heapify_down(pf, pq, 0);
//    }
//    return top;
//}
///**
// * @brief ť���� ������ ��带 �����մϴ�.
// * @param pf Pathfinder
// * @param pq ��� ť
// * @param n ������ ���
// */
//static void pq_remove(Pathfinder* pf, NodePQ* pq, Node* n) {
//    SearchCell* c = cell_of(pf, n); if (!c->in_pq) return;
//    int idx = c->pq_index; pq->size--;
//    if (idx != pq->size) {
//        pq->nodes[idx] = pq->nodes[pq->size];
//        cell_of(pf, pq->nodes[idx])->pq_index = idx;
//        int parent = (idx - 1) / 2;
//        if (idx > 0 && compare_keys(key_of(pf, pq->nodes[idx]), key_of(pf, pq->nodes[parent])) < 0)
//            heapify_up(pf, pq, idx);
//        else heapify_down(pf, pq, idx);
//    }
//    c->in_pq = FALSE; c->pq_index = -1;
//}
//
//// =============================================================================
//// ���� 8: ������ D* Lite ��� Ž�� �˰���
//// =============================================================================
///**
// * @brief �޸���ƽ �Լ�(����ư �Ÿ�).
// * @param a ���� ���
// * @param b ��ǥ ���
// * @return ����ư �Ÿ�
// */
//static double heuristic(const Node* a, const Node* b) {
//    return manhattan_nodes(a, b);
//}
///**
// * @brief D* Lite�� Ű�� ����մϴ�.
// * @param pf Pathfinder
// * @param n ��� ���
// * @return ���� Key
// */
//static Key calculate_key(Pathfinder* pf, const Node* n) {
//    SearchCell* c = cell_of(pf, n);
//    double m = fmin(c->g, c->rhs);
//    return make_key(m + heuristic(pf->start_node, n) + pf->km, m);
//}
///**
// * @brief �� ����� rhs/g�� �����ϰ� OPEN ����Ʈ(��)�� �ϰ��ǰ� �����մϴ�.
// * @param pf Pathfinder
// * @param map �׸��� ��
// * @param am ������Ʈ �Ŵ���(����/��ֹ� �Ǵ�)
// * @param u ������ ���
// */
//static void updateVertex(Pathfinder* pf, GridMap* map, const AgentManager* am, Node* u) {
//    SearchCell* cu = cell_of(pf, u);
//    if (u != pf->goal_node) {
//        double min_rhs = INF;
//        for (int i = 0; i < DIR4_COUNT; i++) {
//            int nx = u->x + DIR4_X[i], ny = u->y + DIR4_Y[i];
//            if (!grid_is_valid_coord(nx, ny)) continue;
//            Node* s = &map->grid[ny][nx];
//            if (!grid_is_node_blocked(map, am, s, pf->agent)) {
//                double gsucc = cell_of(pf, s)->g;
//                double cand = 1.0 + gsucc;
//                if (cand < min_rhs) min_rhs = cand;
//            }
//        }
//        cu->rhs = min_rhs;
//    }
//    if (pq_contains(pf, u)) pq_remove(pf, &pf->pq, u);
//    if (fabs(cu->g - cu->rhs) > 1e-9) {
//        cu->key = calculate_key(pf, u);
//        pq_push(pf, &pf->pq, u);
//    }
//}
///**
// * @brief Pathfinder �ν��Ͻ��� �����ϰ� ��ǥ ��带 OPEN ����Ʈ�� �����մϴ�.
// * @param start ���� ���
// * @param goal ��ǥ ���
// * @return ������ Pathfinder ������
// */
//Pathfinder* pathfinder_create(Node* start, Node* goal, const Agent* agent) {
//    Pathfinder* pf = (Pathfinder*)calloc(1, sizeof(Pathfinder)); if (!pf) return NULL;
//    pq_init(&pf->pq, GRID_WIDTH * GRID_HEIGHT);
//    pf->start_node = start; pf->last_start = start; pf->goal_node = goal; pf->km = 0.0;
//    pf->agent = agent;
//
//    for (int y = 0; y < GRID_HEIGHT; y++)
//        for (int x = 0; x < GRID_WIDTH; x++) {
//            pf->cells[y][x].g = INF; pf->cells[y][x].rhs = INF;
//            pf->cells[y][x].in_pq = FALSE; pf->cells[y][x].pq_index = -1;
//            pf->cells[y][x].key = make_key(INF, INF);
//        }
//
//    if (goal) {
//        SearchCell* cg = &pf->cells[goal->y][goal->x];
//        cg->rhs = 0.0; cg->key = calculate_key(pf, goal);
//        pq_push(pf, &pf->pq, goal);
//    }
//    return pf;
//}
///**
// * @brief Pathfinder ���ҽ��� �����մϴ�.
// * @param pf �ı��� Pathfinder
// */
//void pathfinder_destroy(Pathfinder* pf) { if (pf) { pq_free(&pf->pq); free(pf); } }
//
///**
// * @brief ��ǥ�� �缳���ϰ� ���� ����(g/rhs/OPEN)�� �ʱ�ȭ�մϴ�.
// * @param pf Pathfinder
// * @param new_goal �� ��ǥ ���
// */
//void pathfinder_reset_goal(Pathfinder* pf, Node* new_goal) {
//    pf->goal_node = new_goal; pf->km = 0.0; pf->last_start = pf->start_node;
//    pf->pq.size = 0;
//    for (int y = 0; y < GRID_HEIGHT; y++)
//        for (int x = 0; x < GRID_WIDTH; x++) {
//            pf->cells[y][x].g = INF; pf->cells[y][x].rhs = INF;
//            pf->cells[y][x].in_pq = FALSE; pf->cells[y][x].pq_index = -1;
//            pf->cells[y][x].key = make_key(INF, INF);
//        }
//    if (new_goal) {
//        pf->cells[new_goal->y][new_goal->x].rhs = 0.0;
//        pf->cells[new_goal->y][new_goal->x].key = calculate_key(pf, new_goal);
//        pq_push(pf, &pf->pq, new_goal);
//    }
//}
///**
// * @brief ���� ��� ������ Pathfinder�� �˸��� km�� �����մϴ�.
// * @param pf Pathfinder
// * @param new_start ���ο� ���� ���
// */
//void pathfinder_update_start(Pathfinder* pf, Node* new_start) {
//    if (new_start == NULL) return;
//    if (pf->start_node == NULL) { pf->start_node = new_start; pf->last_start = new_start; return; }
//    pf->km += heuristic(pf->last_start, new_start);
//    pf->last_start = new_start;
//    pf->start_node = new_start;
//}
///**
// * @brief �� ���/���� ��ȭ�� �߻������� Pathfinder�� �����Ͽ� ���� ��带 �����մϴ�.
// * @param pf Pathfinder
// * @param map �׸��� ��
// * @param am ������Ʈ �Ŵ���
// * @param changed ����� ���
// */
//void pathfinder_notify_cell_change(Pathfinder* pf, GridMap* map, const AgentManager* am, Node* changed) {
//    updateVertex(pf, map, am, changed);
//    for (int i = 0; i < DIR4_COUNT; i++) {
//        int px = changed->x + DIR4_X[i], py = changed->y + DIR4_Y[i];
//        if (grid_is_valid_coord(px, py)) updateVertex(pf, map, am, &map->grid[py][px]);
//    }
//}
///**
// * @brief D* Lite �ִ� ��θ� ���������� ����մϴ�.
// * @param pf Pathfinder
// * @param map �׸��� ��
// * @param am ������Ʈ �Ŵ���
// */
//void pathfinder_compute_shortest_path(Pathfinder* pf, GridMap* map, const AgentManager* am) {
//    if (!pf->start_node || !pf->goal_node) return;
//
//    while (TRUE) {
//        Key top = pq_top_key(pf, &pf->pq);
//        SearchCell* cs = cell_of(pf, pf->start_node);
//        Key kstart = calculate_key(pf, pf->start_node);
//
//        if (pf->pq.size == 0 || (compare_keys(top, kstart) >= 0 && fabs(cs->rhs - cs->g) < 1e-9)) break;
//
//        Key k_old = top;
//        Node* u = pq_pop(pf, &pf->pq);
//        SearchCell* cu = cell_of(pf, u);
//        Key k_new = calculate_key(pf, u);
//
//        if (compare_keys(k_old, k_new) < 0) {
//            cu->key = k_new;
//            pq_push(pf, &pf->pq, u);
//        }
//        else if (cu->g > cu->rhs) {
//            cu->g = cu->rhs;
//            for (int i = 0; i < DIR4_COUNT; i++) {
//                int px = u->x + DIR4_X[i], py = u->y + DIR4_Y[i];
//                if (grid_is_valid_coord(px, py))
//                    updateVertex(pf, map, am, &map->grid[py][px]);
//            }
//        }
//        else {
//            cu->g = INF;
//            updateVertex(pf, map, am, u);
//            for (int i = 0; i < DIR4_COUNT; i++) {
//                int px = u->x + DIR4_X[i], py = u->y + DIR4_Y[i];
//                if (grid_is_valid_coord(px, py))
//                    updateVertex(pf, map, am, &map->grid[py][px]);
//            }
//        }
//    }
//}
///**
// * @brief ���� ��忡�� ���� �������� ���� ���� �̿��� ��ȯ�մϴ�.
// *        ������ ��� ��ǥ�� �� �����(����ư) �̿��� �����մϴ�.
// * @param pf Pathfinder
// * @param map �׸��� ��
// * @param am ������Ʈ �Ŵ���
// * @param current ���� ���
// * @return ���� ���� ���(������ current)
// */
//Node* pathfinder_get_next_step(Pathfinder* pf, const GridMap* map, const AgentManager* am, Node* current) {
//    if (!pf->goal_node || !current) return current;
//    SearchCell* cc = cell_of(pf, current);
//    if (cc->g >= INF || current == pf->goal_node) return current;
//
//    double best = INF; Node* bestn = current;
//    double tie_euclid = fabs((double)current->x - (double)pf->goal_node->x) + fabs((double)current->y - (double)pf->goal_node->y);
//
//    for (int i = 0; i < DIR4_COUNT; i++) {
//        int nx = current->x + DIR4_X[i], ny = current->y + DIR4_Y[i];
//        if (!grid_is_valid_coord(nx, ny)) continue;
//        Node* nb = &((GridMap*)map)->grid[ny][nx];
//        if (grid_is_node_blocked(map, am, nb, pf->agent)) continue;
//        double gsucc = cell_of(pf, nb)->g;
//        double cost = 1.0 + gsucc;
//        if (cost < best) {
//            best = cost; bestn = nb;
//            tie_euclid = manhattan_nodes(nb, pf->goal_node);
//        }
//        else if (fabs(cost - best) < 1e-9) {
//            double d = manhattan_nodes(nb, pf->goal_node);
//            if (d < tie_euclid) { bestn = nb; tie_euclid = d; }
//        }
//    }
//    return bestn;
//}
//
//// =============================================================================
//// ���� 9: ��� ��ȹ �� �浹 ȸ�� (WHCA* + WFG/SCC + Partial CBS)
//// =============================================================================
//static void ReservationTable_clear(ReservationTable* r) {
//    for (int t = 0; t <= MAX_WHCA_HORIZON; t++)
//        for (int y = 0; y < GRID_HEIGHT; y++)
//            for (int x = 0; x < GRID_WIDTH; x++)
//                r->occ[t][y][x] = -1;
//}
///**
// * @brief �ð� t=0�� ���� ������Ʈ���� ��ġ�� ���� ���̺� �����մϴ�.
// * @param r ���� ���̺�
// * @param m ������Ʈ �Ŵ���
// */
//static void ReservationTable_seedCurrent(ReservationTable* r, AgentManager* m) {
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* ag = &m->agents[i];
//        if (ag->pos && ag->state != CHARGING) r->occ[0][ag->pos->y][ag->pos->x] = ag->id;
//    }
//}
///**
// * @brief �ð� t���� ��� n�� �̹� ����Ǿ����� Ȯ���մϴ�.
// * @param r ���� ���̺�
// * @param t �ð� (1..H)
// * @param n ���
// * @return ������:1, �ƴϸ� 0
// */
//static int ReservationTable_isOccupied(const ReservationTable* r, int t, const Node* n) {
//    if (t < 0 || t > g_whca_horizon) return TRUE;
//    return r->occ[t][n->y][n->x] != -1;
//}
///**
// * @brief ���� ���̺��� �ش� �ð�/����� ���� ������Ʈ ID�� ��ȯ�մϴ�.
// * @param r ���� ���̺�
// * @param t �ð�
// * @param n ���
// * @return ������Ʈ ID, ������ -1
// */
//static int ReservationTable_getOccupant(const ReservationTable* r, int t, const Node* n) {
//    if (t < 0 || t > g_whca_horizon) return -1;
//    return r->occ[t][n->y][n->x];
//}
///**
// * @brief ���� ���̺� �����ڸ� �����մϴ�.
// * @param r ���� ���̺�
// * @param t �ð�
// * @param n ���
// * @param agent_id ������Ʈ ID
// */
//static void ReservationTable_setOccupant(ReservationTable* r, int t, const Node* n, int agent_id) {
//    if (t < 0 || t > g_whca_horizon) return;
//    r->occ[t][n->y][n->x] = agent_id;
//}
//
//AgentManager* agent_manager_create() {
//    /**
//     * @brief ������Ʈ �Ŵ����� �����ϰ� ��� ������Ʈ �ʵ带 �ʱ�ȭ�մϴ�.
//     * @return ������ AgentManager ������
//     */
//    AgentManager* m = (AgentManager*)calloc(1, sizeof(AgentManager));
//    if (!m) { perror("AgentManager"); exit(1); }
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        m->agents[i].id = i;
//        m->agents[i].symbol = 'A' + i;
//        m->agents[i].state = IDLE;
//        m->agents[i].heading = DIR_NONE;
//        m->agents[i].rotation_wait = 0;
//        m->agents[i].action_timer = 0;
//        m->agents[i].pf = NULL;
//        m->agents[i].stuck_steps = 0;
//        m->agents[i].metrics_task_active = 0;
//        m->agents[i].metrics_task_start_step = 0;
//        m->agents[i].metrics_distance_at_start = 0.0;
//        m->agents[i].metrics_turns_current = 0;
//    }
//    return m;
//}
///**
// * @brief ������Ʈ �Ŵ����� �� ������Ʈ�� Pathfinder�� �����մϴ�.
// * @param m �ı��� AgentManager
// */
//void agent_manager_destroy(AgentManager* m) {
//    if (m) {
//        for (int i = 0; i < MAX_AGENTS; i++) if (m->agents[i].pf) pathfinder_destroy(m->agents[i].pf);
//        free(m);
//    }
//}
//
//// =============================================================================
//// Agent OO-like wrappers (semantics-preserving; use existing logging/metrics)
//// =============================================================================
//void agent_begin_task_park(Agent* ag, ScenarioManager* sc, Logger* lg) {
//    /**
//     * @brief ������Ʈ ���� �۾��� �����ϰ� ��Ʈ���� �ʱ�ȭ�մϴ�.
//     * @param ag ������Ʈ
//     * @param sc �ó����� �Ŵ���
//     * @param lg �ΰ�
//     */
//    if (!ag || !sc) return;
//    ag->state = GOING_TO_PARK;
//    ag->metrics_task_active = 1;
//    ag->metrics_task_start_step = sc->time_step;
//    ag->metrics_distance_at_start = ag->total_distance_traveled;
//    ag->metrics_turns_current = 0;
//    if (lg) logger_log(lg, "[%sTask%s] Agent %c, �ű� ���� �۾� �Ҵ�.", C_CYN, C_NRM, ag->symbol);
//}
//void agent_begin_task_exit(Agent* ag, ScenarioManager* sc, Logger* lg) {
//    /**
//     * @brief ������Ʈ ���� �۾��� �����ϰ� ��Ʈ���� �ʱ�ȭ�մϴ�.
//     * @param ag ������Ʈ
//     * @param sc �ó����� �Ŵ���
//     * @param lg �ΰ�
//     */
//    if (!ag || !sc) return;
//    ag->state = GOING_TO_COLLECT;
//    ag->metrics_task_active = 1;
//    ag->metrics_task_start_step = sc->time_step;
//    ag->metrics_distance_at_start = ag->total_distance_traveled;
//    ag->metrics_turns_current = 0;
//    if (lg) logger_log(lg, "[%sTask%s] Agent %c, �ű� ���� �۾� �Ҵ�.", C_CYN, C_NRM, ag->symbol);
//}
//
//static double calculate_path_cost_tempPF(Agent* ag, Node* goal, GridMap* map, AgentManager* am)
//{
//    /**
//     * @brief �ӽ� Pathfinder�� ����� ���� ��ġ��goal�� ��� ����� �����մϴ�.
//     * @param ag ������Ʈ
//     * @param goal ��ǥ ���
//     * @param map �׸��� ��
//     * @param am ������Ʈ �Ŵ���
//     * @return ��� ���(���� �Ұ� �� INF)
//     */
//     // ���� ����: ��ġ ���̰ų� ��ǥ/���� ������ ��� �Ұ��� ó��
//    if (!ag || !ag->pos || !goal || !map || !am) return INF;
//
//    // ���� ��ġ�� �� ��ǥ�� ��� 0
//    if (ag->pos == goal) return 0.0;
//
//    // �ϵ� ���(��) ��ǥ�� �ٷ� ����
//    if (goal->is_obstacle) return INF;
//
//    Pathfinder* pf = pathfinder_create(ag->pos, goal, ag);
//    if (!pf) return INF;
//
//    pathfinder_compute_shortest_path(pf, map, am);
//    double cost = cell_of(pf, ag->pos)->g;
//
//    pathfinder_destroy(pf);
//
//    // ���� �Ұ� �� INF ����
//    if (cost >= INF * 0.5) return INF;
//    return cost;
//}
//
//// ���� ���� ����: �ĺ� ����Ʈ���� ��� ����� �ּ��� ��带 ����
//// require_parked: -1=����, 0=������ĭ��, 1=����ĭ��
//// check_reserved: 1�̸� Ÿ ������Ʈ ���� ĭ ����
//// toggle_parked_during_eval: ��� �� �� �Ͻ������� is_parked�� ������ ��μ� ��(������)
//// ���� ���� ���� ����
//// - �ĺ� �� ��� ��� �ּ� ��� ����
//// - require_parked: -1=����, 0=������, 1=������
//// - check_reserved: Ÿ ������Ʈ ���� ���� ����
//// - toggle_parked_during_eval: �� ���� is_parked �Ͻ� ����(������)
//static Node* select_best_from_list(Agent* ag, GridMap* map, AgentManager* am,
//    Node** list, int count, int require_parked, int check_reserved, int toggle_parked_during_eval, double* out_best_cost)
//{
//    /**
//     * @brief �ĺ� ��� ����Ʈ���� ��� ����� �ּ��� ��带 �����մϴ�.
//     * @param ag ������Ʈ
//     * @param map �׸��� ��
//     * @param am ������Ʈ �Ŵ���
//     * @param list �ĺ� ��� �迭
//     * @param count �ĺ� ��
//     * @param require_parked -1=����, 0=������, 1=������
//     * @param check_reserved Ÿ ������Ʈ ���� ���� ����(1=����)
//     * @param toggle_parked_during_eval �� �� �Ͻ������� is_parked ���� ����(���� ���ÿ�)
//     * @param out_best_cost ���õ� ��� ��ȯ ������(����)
//     * @return ���� ���(������ NULL)
//     */
//    double best = INF; Node* bestn = NULL;
//    for (int j = 0; j < count; j++) {
//        Node* n = list[j]; if (!n) continue;
//        if (require_parked == 1 && !n->is_parked) continue;
//        if (require_parked == 0 && n->is_parked) continue;
//        if (check_reserved && (n->reserved_by_agent != -1 && n->reserved_by_agent != ag->id)) continue;
//
//        int restored = 0;
//        if (toggle_parked_during_eval && n->is_parked) { n->is_parked = FALSE; restored = 1; }
//        double c = calculate_path_cost_tempPF(ag, n, map, am);
//        if (restored) n->is_parked = TRUE;
//
//        if (c < best) { best = c; bestn = n; }
//    }
//    if (out_best_cost) *out_best_cost = best;
//    return bestn;
//}
//
//// ���� ��ǥ ���� Ÿ�԰� ����
//typedef enum { GOAL_PARKING, GOAL_PARKED_CAR, GOAL_CHARGE } GoalType;
//
//static Node* select_best_goal(Agent* ag, GridMap* map, AgentManager* am, Logger* lg, GoalType type, double* out_cost) {
//    /**
//     * @brief ��ǥ ������(GOAL_PARKING/GOAL_PARKED_CAR/GOAL_CHARGE) ���� ��带 �����մϴ�.
//     * @param ag ������Ʈ
//     * @param map �׸��� ��
//     * @param am ������Ʈ �Ŵ���
//     * @param lg �ΰ�
//     * @param type ��ǥ ����
//     * @param out_cost ���� ��� ��ȯ ������(����)
//     * @return ���� ��ǥ ���
//     */
//    Node** list = NULL; int count = 0; int require_parked = -1; int check_reserved = 1; int toggle_parked = 0;
//    switch (type) {
//    case GOAL_PARKING:
//        list = map->goals; count = map->num_goals; require_parked = 0; toggle_parked = 0; break;
//    case GOAL_PARKED_CAR:
//        list = map->goals; count = map->num_goals; require_parked = 1; toggle_parked = 1; break;
//    case GOAL_CHARGE:
//        list = map->charge_stations; count = map->num_charge_stations; require_parked = -1; toggle_parked = 0; break;
//    }
//    return select_best_from_list(ag, map, am, list, count, require_parked, check_reserved, toggle_parked, out_cost);
//}
//
//static Node* select_best_parking_spot(Agent* ag, GridMap* map, AgentManager* am, Logger* lg) {
//    /**
//     * @brief ���� ������ ���� ��ġ�� �����մϴ�.
//     * @return ���õ� ���(������ NULL)
//     */
//    double best_cost = INF;
//    Node* bestg = select_best_goal(ag, map, am, lg, GOAL_PARKING, &best_cost);
//    if (bestg) logger_log(lg, "[%sPlan%s] Agent %c, ���� ���� (%d,%d) ���� (���: %.1f)",
//        C_CYN, C_NRM, ag->symbol, bestg->x, bestg->y, best_cost);
//    return bestg;
//}
//static Node* select_best_parked_car(Agent* ag, GridMap* map, AgentManager* am, Logger* lg) {
//    /**
//     * @brief ���� ���(������ ����) �� ���� ��ġ�� �����մϴ�.
//     * @return ���õ� ���(������ NULL)
//     */
//    double best_cost = INF;
//    Node* bests = select_best_goal(ag, map, am, lg, GOAL_PARKED_CAR, &best_cost);
//    if (bests) logger_log(lg, "[%sPlan%s] Agent %c, ���� ���� (%d,%d) ���� (���: %.1f)",
//        C_CYN, C_NRM, ag->symbol, bests->x, bests->y, best_cost);
//    return bests;
//}
//static Node* select_best_charge_station(Agent* ag, GridMap* map, AgentManager* am, Logger* lg) {
//    /**
//     * @brief ��� ������ ���� �����Ҹ� �����մϴ�.
//     * @return ���õ� ������ ���(������ NULL)
//     */
//    double best_cost = INF;
//    Node* bests = select_best_goal(ag, map, am, lg, GOAL_CHARGE, &best_cost);
//    if (bests) logger_log(lg, "[%sPlan%s] Agent %c, ������ (%d,%d) ���� (���: %.1f)",
//        C_CYN, C_NRM, ag->symbol, bests->x, bests->y, best_cost);
//    return bests;
//}
//static void ensure_pathfinder_for_agent(Agent* ag) {
//    /**
//     * @brief ������Ʈ�� Pathfinder�� ������ �����ϰ�, ��ǥ ���� �� �缳���մϴ�.
//     * @param ag ������Ʈ
//     */
//    if (!ag->goal) return;
//    if (ag->pf == NULL) {
//        ag->pf = pathfinder_create(ag->pos, ag->goal, ag);
//    }
//    else if (ag->pf->goal_node != ag->goal) {
//        ag->pf->start_node = ag->pos;
//        pathfinder_reset_goal(ag->pf, ag->goal);
//    }
//}
//
//// priority score
//static int priority_score(const Agent* ag) {
//    /**
//     * @brief ������Ʈ ���¿� stuck ������ �켱���� ������ ����մϴ�.
//     * @param ag ������Ʈ
//     * @return ���� ���ϼ��� ���� �켱����
//     */
//    int imp = 0;
//    if (ag->state == RETURNING_WITH_CAR) imp = PRIORITY_RETURNING_WITH_CAR;
//    else if (ag->state == GOING_TO_CHARGE) imp = PRIORITY_GOING_TO_CHARGE;
//    else if (ag->state == GOING_TO_PARK || ag->state == GOING_TO_COLLECT) imp = PRIORITY_MOVING_TASK;
//
//    {
//        int stuck_boost = (ag->stuck_steps >= DEADLOCK_THRESHOLD) ? STUCK_BOOST_HARD : (ag->stuck_steps * STUCK_BOOST_MULT);
//        return imp * 100 + stuck_boost - ag->id;
//    }
//}
//static void sort_agents_by_priority(AgentManager* m, int order[MAX_AGENTS]) {
//    /**
//     * @brief �켱���� ������ ���� ������Ʈ �ε��� �迭�� �������� �����մϴ�.
//     * @param m ������Ʈ �Ŵ���
//     * @param order ���� ���(�ε���) ��� �迭
//     */
//    for (int i = 0; i < MAX_AGENTS; i++) order[i] = i;
//    for (int i = 0; i < MAX_AGENTS; i++)
//        for (int j = i + 1; j < MAX_AGENTS; j++)
//            if (priority_score(&m->agents[order[j]]) > priority_score(&m->agents[order[i]])) {
//                int t = order[i]; order[i] = order[j]; order[j] = t;
//            }
//}
//static void agent_set_goal(Agent* ag, GridMap* map, AgentManager* am, Logger* lg)
//{
//    /**
//     * @brief ������Ʈ ���¿� ���� ������ ��ǥ(����ĭ/����ĭ/������/����)�� �����մϴ�.
//     *        �ʿ�� ���� ������ �����ϰ�, ��ǥ ������ �����մϴ�.
//     * @param ag ������Ʈ
//     * @param map �׸��� ��
//     * @param am ������Ʈ �Ŵ���
//     * @param lg �ΰ�
//     */
//     // 0) ��ġ���� ���� ������Ʈ�� �ƹ� �͵� ���� ����(ũ���� ����)
//    if (!ag || !ag->pos) {
//        ag->goal = NULL;
//        ag->state = IDLE;
//        return;
//    }
//
//    // 1) ���� �Ӱ�ġ ���� �� ���� ��ȯ
//    if (ag->state == RETURNING_HOME_EMPTY &&
//        ag->total_distance_traveled >= DISTANCE_BEFORE_CHARGE) {
//        if (ag->goal) { ag->goal->reserved_by_agent = -1; ag->goal = NULL; }
//        logger_log(lg, "[%sCharge%s] Agent %c ���� �ʿ�! ��ǥ�� �����ҷ� ��ȯ.", C_B_YEL, C_NRM, ag->symbol);
//        ag->state = GOING_TO_CHARGE;
//    }
//
//    // 2) �̹� ��ǥ�� �ְų�, ���/���� ���̸� ����
//    if (ag->state == IDLE || ag->state == CHARGING || ag->goal) return;
//
//    // 3) ���º� ��ǥ ����
//    Node* new_goal = NULL;
//    switch (ag->state) {
//    case GOING_TO_PARK:
//        new_goal = select_best_parking_spot(ag, map, am, lg);
//        break;
//    case RETURNING_HOME_EMPTY:
//    case RETURNING_WITH_CAR:
//    case RETURNING_HOME_MAINTENANCE:
//        new_goal = ag->home_base;
//        break;
//    case GOING_TO_COLLECT:
//        new_goal = select_best_parked_car(ag, map, am, lg);
//        break;
//    case GOING_TO_CHARGE:
//        new_goal = select_best_charge_station(ag, map, am, lg);
//        break;
//    default:
//        break;
//    }
//
//    // 4) ��ǥ ����/���� ó��
//    if (new_goal) {
//        // (�ʿ�� ���� ���� ���� - ���� ��ȯ ���� ������ �̹� ����)
//        if (ag->goal && ag->goal != new_goal) ag->goal->reserved_by_agent = -1;
//
//        ag->goal = new_goal;
//        ag->goal->reserved_by_agent = ag->id;  // ���� ����(������ ����)
//    }
//    else {
//        // ��ǥ�� �� ���� ��� �����ϰ� ��� ��ȯ (��ȯ ���´� ���������� home_base ���� �ø� ���)
//        if (ag->state == RETURNING_HOME_EMPTY ||
//            ag->state == RETURNING_WITH_CAR ||
//            ag->state == RETURNING_HOME_MAINTENANCE) {
//            if (!ag->home_base) {
//                ag->state = IDLE;
//                logger_log(lg, "[%sWarn%s] Agent %c: Ȩ ���̽��� ���� ��� ���·� ��ȯ.", C_B_RED, C_NRM, ag->symbol);
//            }
//        }
//        else {
//            ag->state = IDLE;
//            logger_log(lg, "[%sInfo%s] Agent %c: ���� ��ǥ ����. ���.", C_YEL, C_NRM, ag->symbol);
//        }
//    }
//}
//
//static int best_candidate_order(Pathfinder* pf, const GridMap* map, const AgentManager* am,
//    Node* cur, Node* goal, Node* out[5], int* outN) {
//    /**
//     * @brief ���� ��ġ���� �ĺ�(����+4��)���� ���/�޸���ƽ �������� �����մϴ�.
//     * @param pf Pathfinder
//     * @param map �׸��� ��
//     * @param am ������Ʈ �Ŵ���
//     * @param cur ���� ���
//     * @param goal ��ǥ ���
//     * @param out ���ĵ� �ĺ� ��� �迭(STAY ����)
//     * @param outN �ĺ� �� ���
//     * @return �ĺ� ��
//     */
//    typedef struct { Node* n; double cost; double d; } Cand;
//    Cand cands[5]; int cn = 0;
//
//    double gcur = cell_of(pf, cur)->g;
//    cands[cn++] = (Cand){ cur, gcur + 1e-6, 1e18 };
//
//    for (int k = 0; k < DIR4_COUNT; k++) {
//        int nx = cur->x + DIR4_X[k], ny = cur->y + DIR4_Y[k];
//        if (!grid_is_valid_coord(nx, ny)) continue;
//        Node* nb = &((GridMap*)map)->grid[ny][nx];
//        if (grid_is_node_blocked(map, am, nb, pf->agent)) continue;
//        double gsucc = cell_of(pf, nb)->g;
//        double cost = 1.0 + gsucc;
//        double d = manhattan_nodes(nb, goal);
//        cands[cn++] = (Cand){ nb, cost, d };
//    }
//    for (int a = 0; a < cn; a++) for (int b = a + 1; b < cn; b++) {
//        if (cands[b].cost < cands[a].cost || (fabs(cands[b].cost - cands[a].cost) < 1e-9 && cands[b].d < cands[a].d)) {
//            Cand t = cands[a]; cands[a] = cands[b]; cands[b] = t;
//        }
//    }
//    for (int i = 0; i < cn; i++) out[i] = cands[i].n;
//    *outN = cn;
//    return cn;
//}
//
//// ---- WFG helpers ----
//static void add_wait_edge(WaitEdge* edges, int* cnt, int from, int to, int t, CauseType cause, int x1, int y1, int x2, int y2) {
//    /**
//     * @brief ��� �׷����� ������ �߰��մϴ�(����/���� �浹 ���� ����).
//     * @param edges ���� �迭
//     * @param cnt ���� ����(�߰��Ǹ� ����)
//     * @param from ����ϴ� ������Ʈ ID
//     * @param to ��� ������Ʈ ID
//     * @param t �浹 �ð�(1..H)
//     * @param cause ����(CAUSE_VERTEX/CAUSE_SWAP)
//     * @param x1,y1 ��ġ ����1
//     * @param x2,y2 ��ġ ����2(���� �� ���)
//     */
//    if (*cnt >= MAX_WAIT_EDGES) return;
//    edges[*cnt].from_id = from; edges[*cnt].to_id = to; edges[*cnt].t = t;
//    edges[*cnt].cause = cause; edges[*cnt].x1 = x1; edges[*cnt].y1 = y1; edges[*cnt].x2 = x2; edges[*cnt].y2 = y2;
//    (*cnt)++;
//}
//static int build_scc_mask_from_edges(const WaitEdge* edges, int cnt) {
//    /**
//     * @brief ��� �׷������� SCC(��������) ���� ���θ� ����ũ�� ��ȯ�մϴ�.
//     * @param edges ���� �迭
//     * @param cnt ���� ��
//     * @return SCC�� ���Ե� ������Ʈ ��Ʈ����ũ(������ 0)
//     */
//    int adj[MAX_AGENTS][MAX_AGENTS] = { 0 };
//    for (int i = 0; i < cnt; i++) {
//        int u = edges[i].from_id, v = edges[i].to_id;
//        if (u >= 0 && v >= 0 && u != v) adj[u][v] = 1;
//    }
//    int reach[MAX_AGENTS][MAX_AGENTS] = { 0 };
//    for (int i = 0; i < MAX_AGENTS; i++)
//        for (int j = 0; j < MAX_AGENTS; j++) reach[i][j] = adj[i][j];
//    for (int k = 0; k < MAX_AGENTS; k++)
//        for (int i = 0; i < MAX_AGENTS; i++)
//            for (int j = 0; j < MAX_AGENTS; j++)
//                reach[i][j] = reach[i][j] || (reach[i][k] && reach[k][j]);
//
//    {
//        int mask = 0;
//        for (int i = 0; i < MAX_AGENTS; i++)
//            for (int j = 0; j < MAX_AGENTS; j++)
//                if (i != j && reach[i][j] && reach[j][i]) { mask |= (1 << i); mask |= (1 << j); }
//        return mask;
//    }
//}
//
//// ---- pull-over(����) �ĺ� �� ĭ ã�� ----
//static Node* try_pull_over(const GridMap* map, const ReservationTable* rt, Agent* ag) {
//    /**
//     * @brief �� ƽ ���� ����ִ� ���� ��(���� ����)�� ���Ѽ��� ��ġ�� Ž���մϴ�.
//     * @param map �׸��� ��
//     * @param rt ���� ���̺�
//     * @param ag ������Ʈ
//     * @return �����ϸ� ���� ��ġ, �Ұ� �� ���� ��ġ
//     */
//    for (int k = 0; k < DIR5_COUNT; k++) {
//        int nx = ag->pos->x + DIR5_X[k], ny = ag->pos->y + DIR5_Y[k];
//        if (!grid_is_valid_coord(nx, ny)) continue;
//        Node* nb = (Node*)&map->grid[ny][nx];
//        if (nb == ag->pos) { if (!ReservationTable_isOccupied(rt, 1, nb)) return nb; else continue; }
//        if (nb->is_obstacle || nb->is_parked) continue;
//        // Ÿ ������Ʈ�� ������ ����ĭ�� ���� �ĺ����� ����
//        if (nb->reserved_by_agent != -1 && nb->reserved_by_agent != ag->id) continue;
//        if (!ReservationTable_isOccupied(rt, 1, nb)) return nb;
//    }
//    return ag->pos;
//}
//
//// priority helper
//static int best_in_mask(const AgentManager* m, int mask) {
//    /**
//     * @brief ����ũ�� ���Ե� ������Ʈ �� �켱������ ���� ���� ID�� ��ȯ�մϴ�.
//     * @param m ������Ʈ �Ŵ���
//     * @param mask ��Ʈ����ũ
//     * @return �ְ� �켱���� ������Ʈ ID(������ -1)
//     */
//    int best = -1, bestScore = -999999;
//    for (int i = 0; i < MAX_AGENTS; i++) if (mask & (1 << i)) {
//        int sc = priority_score(&m->agents[i]);
//        if (sc > bestScore) { bestScore = sc; best = i; }
//    }
//    return best;
//}
//
//// ---- Partial-team CBS: low-level ST-A* ----
//static double G_buf[MAX_TOT], F_buf[MAX_TOT];
//static unsigned char OPEN_buf[MAX_TOT], CLOSED_buf[MAX_TOT];
//static int PREV_buf[MAX_TOT];
//
//static int violates_constraint_for(int agent, int t_prev, int x_prev, int y_prev, int x_new, int y_new,
//    const CBSConstraint* cons, int ncons) {
//    /**
//     * @brief �־��� �̵��� CBS ���� ������ �����ϴ��� �˻��մϴ�.
//     * @param agent ������Ʈ ID
//     * @param t_prev ���� �ð�
//     * @param x_prev,y_prev ���� ��ġ
//     * @param x_new,y_new �� ��ġ
//     * @param cons ���� �迭
//     * @param ncons ���� ��
//     * @return ����:1, �ƴϸ� 0
//     */
//    for (int i = 0; i < ncons; i++) {
//        if (cons[i].agent != agent) continue;
//        if (cons[i].is_edge) {
//            if (cons[i].t == t_prev && cons[i].x == x_prev && cons[i].y == y_prev &&
//                cons[i].tox == x_new && cons[i].toy == y_new) return 1;
//        }
//        else {
//            if (cons[i].t == (t_prev + 1) && cons[i].x == x_new && cons[i].y == y_new) return 1;
//        }
//    }
//    return 0;
//}
//static int st_astar_plan_single(int agent_id, GridMap* map, Node* start, Node* goal, int horizon,
//    int ext_occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH],
//    const CBSConstraint* cons, int ncons,
//    Node* out_plan[MAX_WHCA_HORIZON + 1],
//    AgentDir initial_heading) {
//    /**
//     * @brief �ð�-���� A*�� ���� ������Ʈ�� ȣ������ �� ��ȹ�� �����մϴ�.
//     *        ���� �̵� ����(STAY), �ܺ� ����/������ ����մϴ�.
//     * @param agent_id ������Ʈ ID
//     * @param map �׸��� ��
//     * @param start ���� ���
//     * @param goal ��ǥ ���(������ ȣ������ �� ���� ��ġ)
//     * @param horizon WHCA* ȣ������
//     * @param ext_occ �ܺ� ���� ���̺�(�ٸ� ������Ʈ)
//     * @param cons CBS ���� �迭
//     * @param ncons ���� ��
//     * @param out_plan t=0..H ��� ��� �迭
//     * @return ����:1, ����:0
//     */
//    if (!start) return 0;
//    int T = horizon;
//    int W = GRID_WIDTH, H = GRID_HEIGHT;
//    int TOT = (T + 1) * W * H;
//    if (TOT > MAX_TOT) return 0;
//
//    double* g = G_buf;
//    double* f = F_buf;
//    unsigned char* open = OPEN_buf;
//    unsigned char* closed = CLOSED_buf;
//    int* prev = PREV_buf;
//
//    for (int i = 0; i < TOT; i++) { g[i] = INF; f[i] = INF; open[i] = 0; closed[i] = 0; prev[i] = -1; }
//
//#ifndef ST_INDEX
//#define ST_INDEX(t,y,x,width,height) ((t)*(width)*(height) + (y)*(width) + (x))
//#endif
//
//    int sx = start->x, sy = start->y;
//    int gx = goal ? goal->x : sx, gy = goal ? goal->y : sy;
//
//    int start_idx = ST_INDEX(0, sy, sx, W, H);
//    g[start_idx] = 0.0;
//    f[start_idx] = goal ? manhattan_xy(sx, sy, gx, gy) : 0.0;
//    open[start_idx] = 1;
//
//    int best_idx = start_idx; double best_val = f[start_idx];
//
//    while (1) {
//        int cur = -1; double curF = INF;
//        for (int i = 0; i < TOT; i++) if (open[i] && f[i] < curF) { curF = f[i]; cur = i; }
//        if (cur == -1) break;
//        open[cur] = 0; closed[cur] = 1;
//
//        int ct = cur / (W * H);
//        int rem = cur % (W * H);
//        int cy = rem / W, cx = rem % W;
//
//        if (goal && cx == gx && cy == gy) {
//            best_idx = cur; break;
//        }
//        if (f[cur] < best_val) { best_val = f[cur]; best_idx = cur; }
//
//        if (ct == T) continue;
//
//        for (int k = 0; k < DIR5_COUNT; k++) {
//            int nx = cx + DIR5_X[k], ny = cy + DIR5_Y[k];
//            int nt = ct + 1;
//            if (!grid_is_valid_coord(nx, ny)) continue;
//
//            if (ext_occ[nt][ny][nx] != -1) continue;
//
//            if (ext_occ[ct][ny][nx] != -1 && ext_occ[nt][cy][cx] == ext_occ[ct][ny][nx]) continue;
//
//            Node* ncell = &map->grid[ny][nx];
//            if (ncell->is_obstacle) continue;
//
//            if (violates_constraint_for(agent_id, ct, cx, cy, nx, ny, cons, ncons)) continue;
//
//            int nid = ST_INDEX(nt, ny, nx, W, H);
//            if (closed[nid]) continue;
//            {
//                double ng = g[cur] + 1.0;
//                // ȸ�� ������ ����� �ʱ� �̵�(ct==0)���� ����ġ�� �ݿ�
//                if (ct == 0 && !(nx == cx && ny == cy)) {
//                    AgentDir move_heading = dir_from_delta(nx - cx, ny - cy);
//                    if (initial_heading != DIR_NONE) {
//                        int tsteps = dir_turn_steps(initial_heading, move_heading);
//                        if (tsteps == 1) {
//                            ng += (double)TURN_90_WAIT; // 90�� ȸ�� ����ġ
//                        }
//                    }
//                }
//                if (ng + 1e-9 < g[nid]) {
//                    g[nid] = ng;
//                    double h = goal ? manhattan_xy(nx, ny, gx, gy) : 0.0;
//                    f[nid] = ng + h;
//                    prev[nid] = cur;
//                    open[nid] = 1;
//                }
//            }
//        }
//    }
//
//    int path_idx[MAX_WHCA_HORIZON + 1]; int plen = 0;
//    {
//        int cur = best_idx;
//        while (cur != -1 && plen < (MAX_WHCA_HORIZON + 1)) { path_idx[plen++] = cur; cur = prev[cur]; }
//    }
//    if (plen == 0) {
//        for (int t = 0; t <= T; t++) out_plan[t] = start;
//        return 1;
//    }
//    for (int t = 0; t < plen; t++) {
//        int idx = path_idx[plen - 1 - t];
//        int tt = idx / (W * H);
//        int rem = idx % (W * H);
//        int y = rem / W, x = rem % W;
//        if (tt <= T) out_plan[tt] = &map->grid[y][x];
//    }
//    {
//        Node* last = out_plan[plen - 1 <= T ? plen - 1 : T];
//        for (int t = plen; t <= T; t++) out_plan[t] = last;
//    }
//
//    return 1;
//}
//
//// ---- Partial-team CBS high-level ----
//typedef struct {
//    int a, b;
//    int t;
//    int is_edge;
//    int ax, ay, bx, by;
//    int apx, apy, bpx, bpy;
//} CBSConflict;
//
//static double cbs_cost_sum_adv(int ids[], int n,
//    Node* plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1],
//    Node* goals[MAX_AGENTS], int horizon) {
//    const double ALPHA = 1.0, BETA = 0.5, GAMMA = 0.1;
//    double s = 0.0;
//    for (int i = 0; i < n; i++) {
//        int id = ids[i];
//        int moves = 0, waits = 0;
//        for (int t = 1; t <= horizon; t++) {
//            if (plans[id][t] != plans[id][t - 1]) moves++;
//            else waits++;
//        }
//        {
//            double hres = 0.0;
//            if (goals[id]) {
//                Node* last = plans[id][horizon];
//                hres = manhattan_nodes(last, goals[id]);
//            }
//            s += ALPHA * moves + BETA * waits + GAMMA * hres;
//        }
//    }
//    return s;
//}
//static int detect_first_conflict(Node* plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1], int ids[], int n, CBSConflict* out, int horizon) {
//    for (int t = 1; t <= horizon; t++) {
//        for (int i = 0; i < n; i++) for (int j = i + 1; j < n; j++) {
//            int a = ids[i], b = ids[j];
//            Node* a_t = plans[a][t];
//            Node* b_t = plans[b][t];
//            Node* a_tm1 = plans[a][t - 1];
//            Node* b_tm1 = plans[b][t - 1];
//            if (a_t == b_t) {
//                out->a = a; out->b = b; out->t = t; out->is_edge = 0;
//                out->ax = a_t->x; out->ay = a_t->y; out->bx = b_t->x; out->by = b_t->y;
//                out->apx = a_tm1->x; out->apy = a_tm1->y; out->bpx = b_tm1->x; out->bpy = b_tm1->y;
//                return 1;
//            }
//            if (a_t == b_tm1 && b_t == a_tm1) {
//                out->a = a; out->b = b; out->t = t; out->is_edge = 1;
//                out->ax = a_tm1->x; out->ay = a_tm1->y; out->bx = b_tm1->x; out->by = b_tm1->y;
//                out->apx = a_tm1->x; out->apy = a_tm1->y; out->bpx = b_tm1->x; out->bpy = b_tm1->y;
//                return 1;
//            }
//        }
//    }
//    return 0;
//}
//static void copy_ext_occ_without_group(const ReservationTable* base, int group_mask,
//    int out_occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH]) {
//    for (int t = 0; t <= g_whca_horizon; t++)
//        for (int y = 0; y < GRID_HEIGHT; y++)
//            for (int x = 0; x < GRID_WIDTH; x++) {
//                int who = base->occ[t][y][x];
//                if (who != -1 && (group_mask & (1 << who))) out_occ[t][y][x] = -1;
//                else out_occ[t][y][x] = who;
//            }
//}
//
//// --- CBS�� �ּ� �� (C ����) ---
//static void cbs_heap_push(CBSNode* heap, int* hsize, const CBSNode* node) {
//    /**
//     * @brief CBS ��� �ּ� ���� ��带 �����մϴ�.
//     * @param heap �� �迭
//     * @param hsize �� ũ��(���� �� ����)
//     * @param node ������ ���
//     */
//    if (*hsize >= MAX_CBS_NODES) return;
//    heap[*hsize] = *node;
//    int i = *hsize;
//    (*hsize)++;
//    while (i > 0) {
//        int p = (i - 1) / 2;
//        if (heap[p].cost <= heap[i].cost) break;
//        CBSNode tmp = heap[p]; heap[p] = heap[i]; heap[i] = tmp;
//        i = p;
//    }
//}
//static CBSNode cbs_heap_pop(CBSNode* heap, int* hsize) {
//    /**
//     * @brief CBS ��� �ּ� ������ ��Ʈ�� �����Ͽ� ��ȯ�մϴ�.
//     * @param heap �� �迭
//     * @param hsize �� ũ��(����)
//     * @return �˵� CBSNode
//     */
//    CBSNode ret = heap[0];
//    *hsize = *hsize - 1;
//    heap[0] = heap[*hsize];
//    int i = 0;
//    while (1) {
//        int l = 2 * i + 1, r = 2 * i + 2, s = i;
//        if (l < *hsize && heap[l].cost < heap[s].cost) s = l;
//        if (r < *hsize && heap[r].cost < heap[s].cost) s = r;
//        if (s == i) break;
//        CBSNode tmp = heap[s]; heap[s] = heap[i]; heap[i] = tmp;
//        i = s;
//    }
//    return ret;
//}
//
//// Partial CBS (C ����)
//static int run_partial_CBS(AgentManager* m, GridMap* map, Logger* lg,
//    int group_ids[], int group_n, const ReservationTable* base_rt,
//    Node* out_plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1]) {
//    /**
//     * @brief WFG�� ������ �ұ׷쿡 ���� Partial CBS�� �����մϴ�.
//     * @param m ������Ʈ �Ŵ���
//     * @param map �׸��� ��
//     * @param lg �ΰ�
//     * @param group_ids �׷� ������Ʈ ID �迭
//     * @param group_n �׷� ũ��
//     * @param base_rt ��� ���� ���̺�
//     * @param out_plans ��� ��ȹ(t=0..H)
//     * @return ����:1, ����:0
//     */
//    if (group_n <= 1) return 0;
//
//    int group_mask = 0; for (int i = 0; i < group_n; i++) group_mask |= (1 << group_ids[i]);
//
//    // Move large temporaries out of the stack to avoid stack overflow (0xC00000FD)
//    static int ext_occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH];
//    copy_ext_occ_without_group(base_rt, group_mask, ext_occ);
//
//    static CBSNode heap[MAX_CBS_NODES]; int hsize = 0; int expansions = 0;
//
//    CBSNode root; memset(&root, 0, sizeof(root));
//    for (int i = 0; i < group_n; i++) {
//        int id = group_ids[i];
//        Node* plan[MAX_WHCA_HORIZON + 1];
//        if (!st_astar_plan_single(id, map, m->agents[id].pos, m->agents[id].goal, g_whca_horizon, ext_occ,
//            root.cons, root.ncons, plan, m->agents[id].heading)) {
//            g_metrics.cbs_ok_last = 0; g_metrics.cbs_exp_last = expansions; g_metrics.cbs_fail_sum++;
//            return 0;
//        }
//        for (int t = 0; t <= g_whca_horizon; t++) root.plans[id][t] = plan[t];
//    }
//    {
//        Node* goals[MAX_AGENTS] = { 0 };
//        for (int i = 0; i < group_n; i++) goals[group_ids[i]] = m->agents[group_ids[i]].goal;
//        root.cost = cbs_cost_sum_adv(group_ids, group_n, root.plans, goals, g_whca_horizon);
//    }
//    cbs_heap_push(heap, &hsize, &root);
//
//    while (hsize > 0 && expansions < CBS_MAX_EXPANSIONS) {
//        CBSNode cur = cbs_heap_pop(heap, &hsize); expansions++;
//        if (expansions > CBS_MAX_EXPANSIONS) break; // safety guard
//
//        CBSConflict conf;
//        if (!detect_first_conflict(cur.plans, group_ids, group_n, &conf, g_whca_horizon)) {
//            for (int i = 0; i < group_n; i++) {
//                int id = group_ids[i];
//                for (int t = 0; t <= g_whca_horizon; t++) out_plans[id][t] = cur.plans[id][t];
//            }
//            logger_log(lg, "[%sCBS%s] �κ� �� CBS ���� (group=%d agents, expansions=%d).", C_B_GRN, C_NRM, group_n, expansions);
//            g_metrics.cbs_ok_last = 1; g_metrics.cbs_exp_last = expansions; g_metrics.cbs_success_sum++;
//            return 1;
//        }
//
//        for (int branch = 0; branch < 2; branch++) {
//            if (hsize >= MAX_CBS_NODES) break;
//            CBSNode child = cur; // copy by value (heap local); keep heap size bounded
//            if (child.ncons >= MAX_CBS_CONS) continue;
//
//            CBSConstraint c; memset(&c, 0, sizeof(c));
//            if (branch == 0) c.agent = conf.a; else c.agent = conf.b;
//            if (conf.is_edge) {
//                c.is_edge = 1; c.t = conf.t - 1;
//                if (branch == 0) { c.x = conf.apx; c.y = conf.apy; c.tox = conf.bpx; c.toy = conf.bpy; }
//                else { c.x = conf.bpx; c.y = conf.bpy; c.tox = conf.apx; c.toy = conf.apy; }
//            }
//            else {
//                c.is_edge = 0; c.t = conf.t;
//                c.x = conf.ax; c.y = conf.ay;
//            }
//            child.cons[child.ncons++] = c;
//
//            {
//                int ok = 1;
//                for (int i = 0; i < group_n; i++) {
//                    int id = group_ids[i];
//                    Node* plan[MAX_WHCA_HORIZON + 1];
//                    if (!st_astar_plan_single(id, map, m->agents[id].pos, m->agents[id].goal, g_whca_horizon, ext_occ,
//                        child.cons, child.ncons, plan, m->agents[id].heading)) {
//                        ok = 0; break;
//                    }
//                    for (int t = 0; t <= g_whca_horizon; t++) child.plans[id][t] = plan[t];
//                }
//                if (!ok) continue;
//            }
//
//            {
//                Node* goals[MAX_AGENTS] = { 0 };
//                for (int i = 0; i < group_n; i++) goals[group_ids[i]] = m->agents[group_ids[i]].goal;
//                child.cost = cbs_cost_sum_adv(group_ids, group_n, child.plans, goals, g_whca_horizon);
//            }
//
//            cbs_heap_push(heap, &hsize, &child);
//        }
//    }
//    logger_log(lg, "[%sCBS%s] �κ� �� CBS ����(�ð�/�б� �ѵ�). pull-over�� ��ȭ �õ�.", C_B_RED, C_NRM);
//    g_metrics.cbs_ok_last = 0; g_metrics.cbs_exp_last = expansions; g_metrics.cbs_fail_sum++;
//    return 0;
//}
//
//void agent_manager_plan_and_resolve_collisions(AgentManager* m, GridMap* map, Logger* lg, Node* next_pos[MAX_AGENTS]) {
//    /**
//     * @brief ���� �÷���: ��ǥ ���� �� WHCA* ���� �� WFG/SCC �� Partial CBS �� pull-over�� �浹�� ��ȭ�մϴ�.
//     * @param m ������Ʈ �Ŵ���
//     * @param map �׸��� ��
//     * @param lg �ΰ�
//     * @param next_pos �� ������Ʈ�� ���� ��ġ ���
//     */
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* ag = &m->agents[i];
//        g_agent_ops.setGoalIfNeeded(ag, map, m, lg);
//    }
//    for (int i = 0; i < MAX_AGENTS; i++) next_pos[i] = m->agents[i].pos;
//
//    int order[MAX_AGENTS]; sort_agents_by_priority(m, order);
//
//    ReservationTable rt; ReservationTable_clear(&rt); ReservationTable_seedCurrent(&rt, m);
//    WaitEdge wf_edges[MAX_WAIT_EDGES]; int wf_cnt = 0;
//
//    for (int oi = 0; oi < MAX_AGENTS; oi++) {
//        int i = order[oi];
//        Agent* ag = &m->agents[i];
//        if (ag->state == IDLE || ag->state == CHARGING || ag->goal == NULL) continue;
//
//        // ��� Ÿ�̸� ��(��ǥ ĭ���� �۾� ���)�� ���: ȣ������ ��ü ���� ���� �� ���
//        if (ag->action_timer > 0 && ag->pos && ag->goal && ag->pos == ag->goal) {
//            for (int kk = 1; kk <= g_whca_horizon; kk++) {
//                ReservationTable_setOccupant(&rt, kk, ag->pos, ag->id);
//            }
//            next_pos[ag->id] = ag->pos;
//            continue;
//        }
//
//        ensure_pathfinder_for_agent(ag);
//
//        {
//            int goal_was_parked = (ag->state == GOING_TO_COLLECT && ag->goal->is_parked);
//            if (goal_was_parked) ag->goal->is_parked = FALSE;
//
//            if (ag->pf) { pathfinder_update_start(ag->pf, ag->pos); pathfinder_compute_shortest_path(ag->pf, map, m); }
//
//            Node* plan[MAX_WHCA_HORIZON + 1]; plan[0] = ag->pos;
//            Node* cur = ag->pos;
//
//            for (int k = 1; k <= g_whca_horizon; k++) {
//                Node* cand[5]; int cn = 0;
//                best_candidate_order(ag->pf, map, m, cur, ag->pf->goal_node, cand, &cn);
//
//                Node* chosen = cur;
//
//                for (int ci = 0; ci < cn; ci++) {
//                    Node* nb = cand[ci];
//
//                    // ���� �浹: ��� ���� ���
//                    if (ReservationTable_isOccupied(&rt, k, nb)) {
//                        int who = ReservationTable_getOccupant(&rt, k, nb);
//                        if (who != -1) add_wait_edge(wf_edges, &wf_cnt, ag->id, who, k, CAUSE_VERTEX, nb->x, nb->y, 0, 0);
//                        continue;
//                    }
//                    // ���� �浹: ���� ���
//                    {
//                        int who_prev = ReservationTable_getOccupant(&rt, k - 1, nb);
//                        int who_into_cur = ReservationTable_getOccupant(&rt, k, cur);
//                        if (who_prev != -1 && who_prev == who_into_cur) {
//                            add_wait_edge(wf_edges, &wf_cnt, ag->id, who_prev, k, CAUSE_SWAP, cur->x, cur->y, nb->x, nb->y);
//                            continue;
//                        }
//                    }
//
//                    // ù ��° ���� �ĺ��� �����ϵ�, ������ ��ĵ�Ͽ� ���� ������ ����
//                    if (chosen == cur) {
//                        chosen = nb;
//                    }
//                }
//
//                plan[k] = chosen;
//                ReservationTable_setOccupant(&rt, k, chosen, ag->id);
//                cur = chosen;
//
//                if (cur == ag->goal) {
//                    for (int kk = k + 1; kk <= g_whca_horizon; kk++) ReservationTable_setOccupant(&rt, kk, cur, ag->id), plan[kk] = cur;
//                    break;
//                }
//            }
//            next_pos[ag->id] = plan[1];
//
//            if (goal_was_parked) ag->goal->is_parked = TRUE;
//        }
//    }
//
//    {
//        // next_pos ��� ��� �浹�� WFG�� �ݿ��Ͽ� SCC ���� ��ȭ (t=1)
//        for (int i = 0; i < MAX_AGENTS; i++) {
//            if (m->agents[i].state == IDLE || m->agents[i].state == CHARGING || m->agents[i].goal == NULL) continue;
//            for (int j = i + 1; j < MAX_AGENTS; j++) {
//                if (m->agents[j].state == IDLE || m->agents[j].state == CHARGING || m->agents[j].goal == NULL) continue;
//                if (!next_pos[i] || !next_pos[j]) continue;
//                // ���� ���� �̵� �ǵ�: ����� wait ���� �߰�
//                if (next_pos[i] == next_pos[j]) {
//                    add_wait_edge(wf_edges, &wf_cnt, i, j, 1, CAUSE_VERTEX, next_pos[i]->x, next_pos[i]->y, 0, 0);
//                    add_wait_edge(wf_edges, &wf_cnt, j, i, 1, CAUSE_VERTEX, next_pos[j]->x, next_pos[j]->y, 0, 0);
//                }
//                // ���� �ǵ�: ����� swap ���� �߰�
//                else if (next_pos[i] == m->agents[j].pos && next_pos[j] == m->agents[i].pos) {
//                    add_wait_edge(wf_edges, &wf_cnt, i, j, 1, CAUSE_SWAP,
//                        m->agents[i].pos ? m->agents[i].pos->x : -1,
//                        m->agents[i].pos ? m->agents[i].pos->y : -1,
//                        next_pos[i]->x, next_pos[i]->y);
//                    add_wait_edge(wf_edges, &wf_cnt, j, i, 1, CAUSE_SWAP,
//                        m->agents[j].pos ? m->agents[j].pos->x : -1,
//                        m->agents[j].pos ? m->agents[j].pos->y : -1,
//                        next_pos[j]->x, next_pos[j]->y);
//                }
//            }
//        }
//
//        int sccMask = build_scc_mask_from_edges(wf_edges, wf_cnt);
//        g_metrics.wf_edges_last = wf_cnt; g_metrics.wf_edges_sum += wf_cnt;
//        g_metrics.scc_last = (sccMask ? 1 : 0); g_metrics.scc_sum += (sccMask ? 1 : 0);
//
//        if (sccMask) {
//            int group_ids[MAX_CBS_GROUP]; int group_n = 0;
//            for (int i = 0; i < MAX_AGENTS && group_n < MAX_CBS_GROUP; i++) {
//                if ((sccMask & (1 << i)) == 0) continue;
//                if (m->agents[i].state == IDLE || m->agents[i].state == CHARGING || m->agents[i].goal == NULL) continue;
//                if (m->agents[i].action_timer > 0 && m->agents[i].pos && m->agents[i].goal && m->agents[i].pos == m->agents[i].goal) continue; // �۾� ��� ���� ������Ʈ ����
//                group_ids[group_n++] = i;
//            }
//            if (group_n >= 2) {
//                Node* cbs_plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1] = { {0} };
//                int ok = run_partial_CBS(m, map, lg, group_ids, group_n, &rt, cbs_plans);
//                if (ok) {
//                    for (int gi = 0; gi < group_n; gi++) {
//                        int id = group_ids[gi];
//                        if (cbs_plans[id][1]) next_pos[id] = cbs_plans[id][1];
//                    }
//                }
//                else {
//                    int leader = best_in_mask(m, sccMask);
//                    for (int gi = 0; gi < group_n; gi++) {
//                        int id = group_ids[gi];
//                        if (id == leader) continue;
//                        Node* po = try_pull_over(map, &rt, &m->agents[id]);
//                        if (po) next_pos[id] = po;
//                    }
//                    logger_log(lg, "[%sWFG%s] SCC ����: Leader=%c commit, others pull-over.", C_B_YEL, C_NRM, m->agents[leader].symbol);
//                }
//            }
//        }
//        else {
//            // ����: SCC�� ������ ����(��� ���) �� CBS�� 2~3�� �ұ׷쿡 ����
//            int active_ids[MAX_AGENTS]; int active_n = 0;
//            for (int i = 0; i < MAX_AGENTS; i++) {
//                Agent* ag = &m->agents[i];
//                if (ag->state == IDLE || ag->state == CHARGING || ag->goal == NULL) continue;
//                active_ids[active_n++] = i;
//            }
//            int all_wait = 1;
//            for (int ai = 0; ai < active_n; ai++) {
//                int id = active_ids[ai];
//                if (next_pos[id] != m->agents[id].pos) { all_wait = 0; break; }
//            }
//            if (all_wait && active_n >= 2) {
//                int order[MAX_AGENTS]; sort_agents_by_priority(m, order);
//                int group_ids[MAX_CBS_GROUP]; int group_n = 0;
//                for (int oi = 0; oi < MAX_AGENTS && group_n < MAX_CBS_GROUP; oi++) {
//                    int id = order[oi];
//                    Agent* ag = &m->agents[id];
//                    if (ag->state == IDLE || ag->state == CHARGING || ag->goal == NULL) continue;
//                    if (ag->action_timer > 0 && ag->pos && ag->goal && ag->pos == ag->goal) continue; // �۾� ��� ���� ������Ʈ ����
//                    group_ids[group_n++] = id;
//                }
//                if (group_n >= 2) {
//                    Node* cbs_plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1] = { {0} };
//                    int ok = run_partial_CBS(m, map, lg, group_ids, group_n, &rt, cbs_plans);
//                    if (ok) {
//                        for (int gi = 0; gi < group_n; gi++) {
//                            int id = group_ids[gi];
//                            if (cbs_plans[id][1]) next_pos[id] = cbs_plans[id][1];
//                        }
//                        logger_log(lg, "[%sCBS%s] Deadlock fallback CBS ���� (group=%d).", C_B_CYN, C_NRM, group_n);
//                    }
//                    else {
//                        // ������ ����: ���� 1�븸 ����, ������ ���� pull-over
//                        int leader = best_in_mask(m, sccMask ? sccMask : 0x3FF);
//                        for (int gi = 0; gi < group_n; gi++) {
//                            int id = group_ids[gi];
//                            if (id == leader) continue;
//                            Node* po = try_pull_over(map, &rt, &m->agents[id]);
//                            if (po) next_pos[id] = po;
//                        }
//                        logger_log(lg, "[%sWFG%s] Deadlock fallback: leader-only move, others pull-over.", C_B_YEL, C_NRM);
//                    }
//                }
//            }
//        }
//    }
//
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        for (int j = i + 1; j < MAX_AGENTS; j++) {
//            if (m->agents[i].state == IDLE || m->agents[j].state == IDLE ||
//                m->agents[i].state == CHARGING || m->agents[j].state == CHARGING) continue;
//
//            if (next_pos[i] == next_pos[j]) {
//                // ���� ���� ������ ���� ���� �������� �׽� �켱
//                if ((m->agents[i].state == GOING_TO_PARK && m->agents[j].state == RETURNING_HOME_EMPTY) ||
//                    (m->agents[j].state == GOING_TO_PARK && m->agents[i].state == RETURNING_HOME_EMPTY)) {
//                    if (m->agents[i].state == RETURNING_HOME_EMPTY) {
//                        logger_log(lg, "[%sAvoid%s] ���� �浹(����>����): Agent %c ���.", C_B_RED, C_NRM, m->agents[i].symbol);
//                        next_pos[i] = m->agents[i].pos;
//                    }
//                    else {
//                        logger_log(lg, "[%sAvoid%s] ���� �浹(����>����): Agent %c ���.", C_B_RED, C_NRM, m->agents[j].symbol);
//                        next_pos[j] = m->agents[j].pos;
//                    }
//                }
//                else {
//                    int pi = priority_score(&m->agents[i]);
//                    int pj = priority_score(&m->agents[j]);
//                    if (pi >= pj) { logger_log(lg, "[%sAvoid%s] ���� �浹 �� Agent %c ���.", C_B_RED, C_NRM, m->agents[j].symbol); next_pos[j] = m->agents[j].pos; }
//                    else { logger_log(lg, "[%sAvoid%s] ���� �浹 �� Agent %c ���.", C_B_RED, C_NRM, m->agents[i].symbol); next_pos[i] = m->agents[i].pos; }
//                }
//            }
//            else if (next_pos[i] == m->agents[j].pos && next_pos[j] == m->agents[i].pos) {
//                // ���� ���� ������ ���� ���� �������� �׽� �켱
//                if ((m->agents[i].state == GOING_TO_PARK && m->agents[j].state == RETURNING_HOME_EMPTY) ||
//                    (m->agents[j].state == GOING_TO_PARK && m->agents[i].state == RETURNING_HOME_EMPTY)) {
//                    if (m->agents[i].state == RETURNING_HOME_EMPTY) {
//                        logger_log(lg, "[%sAvoid%s] ���� �浹(����>����): Agent %c ���.", C_B_RED, C_NRM, m->agents[i].symbol);
//                        next_pos[i] = m->agents[i].pos;
//                    }
//                    else {
//                        logger_log(lg, "[%sAvoid%s] ���� �浹(����>����): Agent %c ���.", C_B_RED, C_NRM, m->agents[j].symbol);
//                        next_pos[j] = m->agents[j].pos;
//                    }
//                }
//                else {
//                    int pi = priority_score(&m->agents[i]);
//                    int pj = priority_score(&m->agents[j]);
//                    if (pi >= pj) { logger_log(lg, "[%sAvoid%s] ���� �浹 �� Agent %c ���.", C_B_RED, C_NRM, m->agents[j].symbol); next_pos[j] = m->agents[j].pos; }
//                    else { logger_log(lg, "[%sAvoid%s] ���� �浹 �� Agent %c ���.", C_B_RED, C_NRM, m->agents[i].symbol); next_pos[i] = m->agents[i].pos; }
//                }
//            }
//        }
//    }
//
//    // ȣ������ �ڵ� ����
//    WHCA_adjustHorizon(wf_cnt, g_metrics.scc_last, lg);
//}
//
//// ��� Pathfinder�� ȯ�� ��ȭ ����
//static void broadcast_cell_change(AgentManager* am, GridMap* map, Node* changed) {
//    /**
//     * @brief ��� Pathfinder�� Ư�� ���� ȯ�� ��ȭ�� �����մϴ�.
//     * @param am ������Ʈ �Ŵ���
//     * @param map �׸��� ��
//     * @param changed ����� ���
//     */
//    if (!map || !changed) return;
//    for (int a = 0; a < MAX_AGENTS; a++) {
//        if (am->agents[a].pf) pathfinder_notify_cell_change(am->agents[a].pf, map, am, changed);
//    }
//}
//
//void agent_manager_update_state_after_move(AgentManager* m, ScenarioManager* sc, GridMap* map, Logger* lg, Simulation* sim) {
//    /**
//     * @brief �̵� ���� �� ��ǥ ����/�۾� ���/���� ���̸� ó���ϰ� ��Ʈ��/������ �����մϴ�.
//     * @param m ������Ʈ �Ŵ���
//     * @param sc �ó����� �Ŵ���
//     * @param map �׸��� ��
//     * @param lg �ΰ�
//     * @param sim �ùķ��̼�(���� ��Ʈ�� �ݿ�)
//     */
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* ag = &m->agents[i];
//        if (ag->state == IDLE || ag->state == CHARGING || !ag->goal || ag->pos != ag->goal) continue;
//
//        // ����/���� ���� �� ��� ����
//        if (ag->state == GOING_TO_PARK || ag->state == GOING_TO_COLLECT) {
//            if (ag->action_timer <= 0) {
//                ag->action_timer = TASK_ACTION_TICKS;
//                logger_log(lg, "[%sTask%s] Agent %c, %s �۾� ��� %dtick.", C_YEL, C_NRM, ag->symbol,
//                    ag->state == GOING_TO_PARK ? "����" : "����", ag->action_timer);
//                continue; // ��� ����: ��� �Ϸ� ó������ ����
//            }
//            else {
//                ag->action_timer--;
//                if (ag->action_timer > 0) {
//                    continue; // ��� ���� ��
//                }
//                // action_timer�� 0�� �Ǿ����Ƿ� �Ʒ����� �Ϸ� ó���� ����
//            }
//        }
//
//        Node* reached = ag->goal;
//        if (ag->state != GOING_TO_CHARGE) reached->reserved_by_agent = -1;
//        ag->goal = NULL;
//
//        switch (ag->state) {
//        case GOING_TO_PARK:
//            reached->is_parked = TRUE; m->total_cars_parked++;
//            broadcast_cell_change(m, map, reached);
//            logger_log(lg, "[%sPark%s] Agent %c, ���� �Ϸ� at (%d,%d).", C_GRN, C_NRM, ag->symbol, reached->x, reached->y);
//            if (sc->mode == MODE_CUSTOM && sc->current_phase_index < sc->num_phases &&
//                sc->phases[sc->current_phase_index].type == PARK_PHASE) {
//                sc->tasks_completed_in_phase++;
//                if (sim) {
//                    int phase_idx = sc->current_phase_index;
//                    if (phase_idx >= 0 && phase_idx < MAX_PHASES) {
//                        sim->phase_completed_tasks[phase_idx]++;
//                    }
//                    sim->tasks_completed_total++;
//                }
//            }
//            else if (sim) {
//                sim->tasks_completed_total++;
//            }
//            ag->state = RETURNING_HOME_EMPTY;
//            if (ag->pf) { pathfinder_destroy(ag->pf); ag->pf = NULL; }
//            break;
//        case RETURNING_HOME_EMPTY:
//            logger_log(lg, "[%sInfo%s] Agent %c, ���� �۾� �� ���� ���� �Ϸ�.", C_CYN, C_NRM, ag->symbol);
//            ag->state = IDLE;
//            if (ag->pf) { pathfinder_destroy(ag->pf); ag->pf = NULL; }
//            // --- metrics aggregate for PARK cycle ---
//            metrics_finalize_task_if_active(sim, ag);
//            break;
//        case GOING_TO_COLLECT:
//            logger_log(lg, "[%sExit%s] Agent %c, ���� ���� at (%d,%d).", C_YEL, C_NRM, ag->symbol, reached->x, reached->y);
//            reached->is_parked = FALSE; m->total_cars_parked--;
//            broadcast_cell_change(m, map, reached);
//            ag->state = RETURNING_WITH_CAR;
//            if (ag->pf) { pathfinder_destroy(ag->pf); ag->pf = NULL; }
//            break;
//        case RETURNING_WITH_CAR:
//            logger_log(lg, "[%sExit%s] Agent %c, ���� ���� �Ϸ�.", C_GRN, C_NRM, ag->symbol);
//            if (sc->mode == MODE_CUSTOM && sc->current_phase_index < sc->num_phases &&
//                sc->phases[sc->current_phase_index].type == EXIT_PHASE) {
//                sc->tasks_completed_in_phase++;
//                if (sim) {
//                    int phase_idx = sc->current_phase_index;
//                    if (phase_idx >= 0 && phase_idx < MAX_PHASES) {
//                        sim->phase_completed_tasks[phase_idx]++;
//                    }
//                    sim->tasks_completed_total++;
//                }
//            }
//            else if (sim) {
//                sim->tasks_completed_total++;
//            }
//            ag->state = IDLE;
//            if (ag->pf) { pathfinder_destroy(ag->pf); ag->pf = NULL; }
//            // --- metrics aggregate for EXIT cycle ---
//            metrics_finalize_task_if_active(sim, ag);
//            break;
//        case GOING_TO_CHARGE:
//            logger_log(lg, "[%sCharge%s] Agent %c, ���� ����. (%d steps)", C_B_YEL, C_NRM, ag->symbol, CHARGE_TIME);
//            ag->state = CHARGING; ag->charge_timer = CHARGE_TIME;
//            if (ag->pos) broadcast_cell_change(m, map, ag->pos);
//            break;
//        case RETURNING_HOME_MAINTENANCE:
//            logger_log(lg, "[%sInfo%s] Agent %c, ���� �� ���� ���� �Ϸ�.", C_CYN, C_NRM, ag->symbol);
//            ag->state = IDLE;
//            if (ag->pf) { pathfinder_destroy(ag->pf); ag->pf = NULL; }
//            break;
//        default: break;
//        }
//        ag->stuck_steps = 0;
//    }
//}
//void agent_manager_update_charge_state(AgentManager* m, GridMap* map, Logger* lg) {
//    /**
//     * @brief ���� ���� ������Ʈ�� Ÿ�̸Ӹ� ���ҽ�Ű�� �Ϸ� �� ���¸� ��ȯ�մϴ�.
//     * @param m ������Ʈ �Ŵ���
//     * @param map �׸��� ��
//     * @param lg �ΰ�
//     */
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* ag = &m->agents[i];
//        if (ag->state == CHARGING) {
//            ag->charge_timer--;
//            if (ag->charge_timer <= 0) {
//                logger_log(lg, "[%sCharge%s] Agent %c ���� �Ϸ�.", C_B_GRN, C_NRM, ag->symbol);
//                ag->total_distance_traveled = 0.0;
//                ag->state = RETURNING_HOME_MAINTENANCE;
//                if (ag->pos) ag->pos->reserved_by_agent = -1;
//                if (ag->pos) broadcast_cell_change(m, map, ag->pos);
//                ag->goal = NULL;
//                if (ag->pf) { pathfinder_destroy(ag->pf); ag->pf = NULL; }
//                ag->stuck_steps = 0;
//            }
//        }
//    }
//}
//
//// --- WHCA* ȣ������ �ڵ� ���� ---
//// �浹 ������ ���� ȣ�������� �������� ����
//static void WHCA_adjustHorizon(int wf_edges, int scc, Logger* lg) {
//    /**
//     * @brief �ֱ� �浹 ��ǥ�� ������� WHCA* ȣ�������� �������� �����մϴ�.
//     * @param wf_edges ������ ��� ���� ��
//     * @param scc SCC ���� ����(0/1)
//     * @param lg �ΰ�
//     */
//    g_conflict_score = (int)(g_conflict_score * 0.6) + wf_edges + (scc ? 5 : 0);
//    {
//        int oldH = g_whca_horizon;
//        const int HI = 24;
//        const int LO = 10;
//
//        if (g_conflict_score > HI && g_whca_horizon < MAX_WHCA_HORIZON) g_whca_horizon += 2;
//        else if (g_conflict_score < LO && g_whca_horizon > MIN_WHCA_HORIZON) g_whca_horizon -= 2;
//
//        if (g_whca_horizon < MIN_WHCA_HORIZON) g_whca_horizon = MIN_WHCA_HORIZON;
//        if (g_whca_horizon > MAX_WHCA_HORIZON) g_whca_horizon = MAX_WHCA_HORIZON;
//
//        if (oldH != g_whca_horizon) {
//            logger_log(lg, "[%sWHCA*%s] Horizon ����: %d �� %d (score=%d)", C_B_CYN, C_NRM, oldH, g_whca_horizon, g_conflict_score);
//        }
//        g_metrics.whca_h = g_whca_horizon;
//    }
//}
//
//// =============================================================================
//// ���� 9-��ü: ��ü �÷��� (�ܼ� A* / �⺻ D* Lite)
//// =============================================================================
//
//// A* ��� ��ȹ �� �浹 �ذ� (�ܼ� �� ���� ��ȹ)
//void agent_manager_plan_and_resolve_collisions_astar(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) {
//    /**
//     * @brief �ܼ� A* ��� �� ���� ��ȹ + �켱���� �浹 �ؼҸ� �����մϴ�.
//     * @param manager ������Ʈ �Ŵ���
//     * @param map �׸��� ��
//     * @param logger �ΰ�
//     * @param next_pos ���� ��ġ ���
//     */
//     // 1. ��ǥ ���� �� next_pos �ʱ�ȭ (��� ������Ʈ�� ���� ��ġ���� ���)
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* agent = &manager->agents[i];
//        g_agent_ops.setGoalIfNeeded(agent, map, manager, logger);
//        next_pos[i] = agent->pos;
//    }
//
//    // 2. �켱���� ��� ���� ��ȹ (ȸ�� ���� ����)
//    int order[MAX_AGENTS];
//    sort_agents_by_priority(manager, order);
//    for (int oi = 0; oi < MAX_AGENTS; oi++) {
//        int i = order[oi];
//        Agent* agent = &manager->agents[i];
//        Node* current_pos = agent->pos;
//
//        // �̹� ȸ�� �Ǵ� �۾� ��� ���� ���, ��ȹ ���� ��� Ȯ��
//        if (agent->rotation_wait > 0) {
//            agent->rotation_wait--;
//            continue; // next_pos[i]�� �̹� current_pos�� ������
//        }
//        if (agent->action_timer > 0) {
//            continue; // �۾� Ÿ�̸Ӱ� �ִ� ������Ʈ�� �������� ����
//        }
//
//        // ��ȹ�� �ʿ� ���� ���¸� �ǳʶ�
//        if (agent->state == IDLE || agent->state == CHARGING || agent->goal == NULL || !current_pos) {
//            continue;
//        }
//
//        // 3. A*�� �̻����� ���� ����(desired_move) ��ȹ (�켱���� ���� ������Ʈ�� next_pos, ���� ������Ʈ�� ���� pos�� �ӽ� ��ֹ��� ���)
//        Node* desired_move = current_pos;
//        {
//            TempMarkContext ctx; temp_context_init(&ctx, NULL, map, manager, 0);
//            for (int h = 0; h < oi; h++) {
//                int hid = order[h];
//                if (next_pos[hid]) temp_context_mark(&ctx, next_pos[hid]);
//            }
//            for (int l = oi + 1; l < MAX_AGENTS; l++) {
//                int lid = order[l];
//                if (manager->agents[lid].pos) temp_context_mark(&ctx, manager->agents[lid].pos);
//            }
//
//            int goal_was_parked = (agent->state == GOING_TO_COLLECT && agent->goal->is_parked);
//            if (goal_was_parked) agent->goal->is_parked = FALSE;
//
//            Pathfinder* pf = g_pf_factory.create(agent->pos, agent->goal);
//            pathfinder_compute_shortest_path(pf, map, manager);
//            desired_move = pathfinder_get_next_step(pf, map, manager, agent->pos);
//            g_pf_factory.destroy(pf);
//
//            if (goal_was_parked) agent->goal->is_parked = TRUE;
//            temp_context_cleanup(&ctx);
//        }
//
//        // 4. ȸ�� ������ �����Ͽ� ���� ���� ����(next_pos[i]) Ȯ��
//        agent_apply_rotation_and_step(agent, current_pos, desired_move, &next_pos[i]);
//    }
//
//    // 5. ���� �浹 ���� (���� �Լ�)
//    resolve_conflicts_by_order(manager, order, next_pos);
//}
//
//void agent_manager_plan_and_resolve_collisions_dstar_basic(AgentManager* m, GridMap* map, Logger* lg, Node* next_pos[MAX_AGENTS]) {
//    /**
//     * @brief �⺻ D* Lite ��� �� ���� ��ȹ + �켱���� �浹 �ؼҸ� �����մϴ�.
//     * @param m ������Ʈ �Ŵ���
//     * @param map �׸��� ��
//     * @param lg �ΰ�
//     * @param next_pos ���� ��ġ ���
//     */
//     // 1. ��ǥ ���� �� next_pos �ʱ�ȭ
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* ag = &m->agents[i];
//        g_agent_ops.setGoalIfNeeded(ag, map, m, lg);
//        next_pos[i] = ag->pos;
//    }
//
//    // 2. �켱���� ��� ���� ��ȹ (ȸ�� ���� ����)
//    int order[MAX_AGENTS];
//    sort_agents_by_priority(m, order);
//    for (int oi = 0; oi < MAX_AGENTS; oi++) {
//        int i = order[oi];
//        Agent* ag = &m->agents[i];
//        Node* current_pos = ag->pos;
//
//        // �̹� ȸ�� �Ǵ� �۾� ��� ���� ���, ��ȹ ���� ��� Ȯ��
//        if (ag->rotation_wait > 0) {
//            ag->rotation_wait--;
//            continue;
//        }
//        if (ag->action_timer > 0) {
//            continue;
//        }
//
//        // ��ȹ�� �ʿ� ���� ���¸� �ǳʶ�
//        if (ag->state == IDLE || ag->state == CHARGING || ag->goal == NULL || !current_pos) {
//            continue;
//        }
//
//        // 3. D* Lite�� �̻����� ���� ����(desired_move) ��ȹ (�켱���� ���� ������Ʈ�� next_pos, ���� ������Ʈ�� ���� pos�� �ӽ� ��ֹ��� ���)
//        Node* desired_move = current_pos;
//        {
//            if (!ag->pf) {
//                ag->pf = pathfinder_create(ag->pos, ag->goal, ag);
//            }
//            else if (ag->pf->goal_node != ag->goal) {
//                ag->pf->start_node = ag->pos;
//                pathfinder_reset_goal(ag->pf, ag->goal);
//            }
//
//            TempMarkContext ctx; temp_context_init(&ctx, ag->pf, map, m, 1);
//            for (int h = 0; h < oi; h++) {
//                int hid = order[h];
//                if (next_pos[hid]) temp_context_mark(&ctx, next_pos[hid]);
//            }
//            for (int l = oi + 1; l < MAX_AGENTS; l++) {
//                int lid = order[l];
//                if (m->agents[lid].pos) temp_context_mark(&ctx, m->agents[lid].pos);
//            }
//
//            int goal_was_parked = (ag->state == GOING_TO_COLLECT && ag->goal->is_parked);
//            if (goal_was_parked) { ag->goal->is_parked = FALSE; if (ag->pf) pathfinder_notify_cell_change(ag->pf, map, m, ag->goal); }
//
//            if (ag->pf) {
//                pathfinder_update_start(ag->pf, ag->pos);
//                pathfinder_compute_shortest_path(ag->pf, map, m);
//                desired_move = pathfinder_get_next_step(ag->pf, map, m, ag->pos);
//            }
//
//            if (goal_was_parked) { ag->goal->is_parked = TRUE; if (ag->pf) pathfinder_notify_cell_change(ag->pf, map, m, ag->goal); }
//            temp_context_cleanup(&ctx);
//        }
//
//        // 4. ȸ�� ���� �����Ͽ� ���� ���� ���� Ȯ��
//        agent_apply_rotation_and_step(ag, current_pos, desired_move, &next_pos[i]);
//    }
//
//    // 5. ���� �浹 ���� (���� �Լ�)
//    resolve_conflicts_by_order(m, order, next_pos);
//}
//// =============================================================================
//// ���� 9-����: �÷��� ���� ���� (���� ����)
//// =============================================================================
//static void planner_plan_default(AgentManager* am, GridMap* map, Logger* lg, Node* next_pos[MAX_AGENTS]) {
//    /**
//     * @brief ���� �⺻ �÷���(WHCA*+D*Lite+WFG+CBS)�� ȣ���մϴ�.
//     */
//    agent_manager_plan_and_resolve_collisions(am, map, lg, next_pos);
//}
//static void planner_plan_astar(AgentManager* am, GridMap* map, Logger* lg, Node* next_pos[MAX_AGENTS]) {
//    /**
//     * @brief �ܼ� A* �÷��ʸ� ȣ���մϴ�.
//     */
//    agent_manager_plan_and_resolve_collisions_astar(am, map, lg, next_pos);
//}
//static void planner_plan_dstar(AgentManager* am, GridMap* map, Logger* lg, Node* next_pos[MAX_AGENTS]) {
//    /**
//     * @brief �⺻ D* Lite �÷��ʸ� ȣ���մϴ�.
//     */
//    agent_manager_plan_and_resolve_collisions_dstar_basic(am, map, lg, next_pos);
//}
//
//static Planner planner_make_default(void) {
//    /**
//     * @brief �⺻ �÷��� vtable�� �����մϴ�.
//     */
//    Planner p; p.vtbl.plan_step = planner_plan_default; return p;
//}
//static Planner planner_make_astar(void) {
//    /**
//     * @brief A* �÷��� vtable�� �����մϴ�.
//     */
//    Planner p; p.vtbl.plan_step = planner_plan_astar; return p;
//}
//static Planner planner_make_dstar(void) {
//    /**
//     * @brief D* Lite �÷��� vtable�� �����մϴ�.
//     */
//    Planner p; p.vtbl.plan_step = planner_plan_dstar; return p;
//}
//
//static Planner planner_from_pathalgo(PathAlgo algo) {
//    /**
//     * @brief ������ ������ ���� ������ �÷��� vtable�� ��ȯ�մϴ�.
//     * @param algo ���� �˰���
//     * @return Planner ����ü(vtable ����)
//     */
//    switch (algo) {
//    case PATHALGO_ASTAR_SIMPLE: return planner_make_astar();
//    case PATHALGO_DSTAR_BASIC:  return planner_make_dstar();
//    case PATHALGO_DEFAULT:
//    default:                    return planner_make_default();
//    }
//}
//// --- ���� �浹 �ذ� (�켱���� �迭�� ���� ���� �켱���� ������Ʈ�� ���) ---
//// �켱���� �迭�� �������� �浹 �� �ļ��� ������Ʈ�� ����Ŵ
//static void resolve_conflicts_by_order(const AgentManager* m, const int order[MAX_AGENTS], Node* next_pos[MAX_AGENTS]) {
//    /**
//     * @brief �켱���� �迭�� ���� �浹 �� �ļ��� ������Ʈ�� ����ŵ�ϴ�.
//     * @param m ������Ʈ �Ŵ���
//     * @param order �켱���� ���� ���� �ε��� �迭
//     * @param next_pos ��ȹ�� ���� ��ġ �迭(������)
//     */
//    for (int oi = 0; oi < MAX_AGENTS; oi++) {
//        int i = order[oi];
//        for (int oj = oi + 1; oj < MAX_AGENTS; oj++) {
//            int j = order[oj];
//            if (!next_pos[i] || !next_pos[j]) continue;
//            if (next_pos[i] == next_pos[j] ||
//                (next_pos[i] == m->agents[j].pos && next_pos[j] == m->agents[i].pos)) {
//                next_pos[j] = ((AgentManager*)m)->agents[j].pos;
//            }
//            // �߰� ��Ģ: ���� ���� Ÿ ������Ʈ�� ���� ĭ������ ���� ����
//            else if (next_pos[i] == m->agents[j].pos && next_pos[j] == m->agents[j].pos) {
//                next_pos[i] = ((AgentManager*)m)->agents[i].pos;
//            }
//        }
//    }
//}
//// =============================================================================
//// ���� 10: �ùķ��̼� �ٽ� (�� ���� ����)
//// =============================================================================
///**
// * @brief ������ �и���(ms) ���� ����մϴ�.
// * @param ms ��� �ð�(ms)
// */
//static void do_ms_pause(int ms) { sleep_ms(ms); }
//
///**
// * @brief ����� ����(Custom) �ó������� ��ȭ������ �����մϴ�.
// * @param sim �ùķ��̼� �ν��Ͻ�
// * @return ����:1, ���:0
// */
//static int simulation_setup_custom_scenario(Simulation* sim) {
//    ScenarioManager* s = sim->scenario_manager;
//
//    printf(C_B_WHT "--- ����� ���� �ó����� ���� ---\n" C_NRM);
//    s->num_phases = get_integer_input(C_YEL "�� �ܰ� ���� �Է�(1-20, 0=���): " C_NRM, 0, MAX_PHASES);
//    if (s->num_phases == 0) return 0;
//
//    int max_per_phase = (sim->map && sim->map->num_goals > 0) ? sim->map->num_goals : 100000;
//
//    for (int i = 0; i < s->num_phases; i++) {
//        printf(C_B_CYN "\n--- %d/%d �ܰ� ���� ---\n" C_NRM, i + 1, s->num_phases);
//        printf("a. %s����%s\n", C_YEL, C_NRM);
//        printf("b. %s����%s\n", C_CYN, C_NRM);
//        char c = get_char_input("�ܰ� ���� ����: ", "ab");
//
//        char prompt[64];
//        snprintf(prompt, sizeof(prompt), "�� �ܰ� ���� �� (1~%d): ", max_per_phase);
//        s->phases[i].task_count = get_integer_input(prompt, 1, max_per_phase);
//
//        if (c == 'a') { s->phases[i].type = PARK_PHASE;  snprintf(s->phases[i].type_name, sizeof(s->phases[i].type_name), "����"); }
//        else { s->phases[i].type = EXIT_PHASE;  snprintf(s->phases[i].type_name, sizeof(s->phases[i].type_name), "����"); }
//
//        printf(C_GRN "%d�ܰ� ���� �Ϸ�: %s %d��.\n" C_NRM, i + 1, s->phases[i].type_name, s->phases[i].task_count);
//    }
//    printf(C_B_GRN "\n--- �ó����� ���� �Ϸ� ---\n" C_NRM);
//    do_ms_pause(1500);
//    return 1;
//}
//
///**
// * @brief �ǽð� ����� ��û Ȯ��(����/����)�� �����մϴ�.
// * @param s �ó����� �Ŵ���
// * @return ����:1
// */
//static int simulation_setup_realtime(ScenarioManager* s) {
//    printf(C_B_WHT "--- �ǽð� �ùķ��̼� ���� ---\n" C_NRM);
//    while (TRUE) {
//        s->park_chance = get_integer_input("\n���� ��û Ȯ��(0~100): ", 0, 100);
//        s->exit_chance = get_integer_input("���� ��û Ȯ��(0~100): ", 0, 100);
//        if (s->park_chance + s->exit_chance <= 100) break;
//        printf(C_B_RED "���� 100�� ���� �� �����ϴ�.\n" C_NRM);
//    }
//    printf(C_B_GRN "\n���� �Ϸ�: ���� %d%%, ���� %d%%\n" C_NRM, s->park_chance, s->exit_chance);
//    do_ms_pause(1500); return 1;
//}
///**
// * @brief �ùķ��̼� �ӵ�(���)�� �����ϰ� sleep ������ ����մϴ�.
// * @param s �ó����� �Ŵ���
// * @return ����:1
// */
//static int simulation_setup_speed(ScenarioManager* s) {
//    printf(C_B_WHT "\n--- �ùķ��̼� �ӵ� ���� ---\n" C_NRM);
//
//    // 0.0 ���(= ������), ��� ��� 0ms ����
//    s->speed_multiplier = get_float_input("��� (0.0=������ ~ 10000.0): ", 0.0f, MAX_SPEED_MULTIPLIER);
//    if (s->speed_multiplier <= 0.0f) {
//        s->simulation_speed = 0;          // 0ms sleep
//    }
//    else {
//        s->simulation_speed = (int)(100.0f / s->speed_multiplier);
//        if (s->simulation_speed < 0) s->simulation_speed = 0;
//    }
//
//    printf(C_B_GRN "\n--- %.1fx ������� �����մϴ�... ---\n" C_NRM, s->speed_multiplier);
//    do_ms_pause(1500);
//    return 1;
//}
//
//
//// �� Map ���� �ܰ�
///**
// * @brief ���� �� ����� ��(1~5)�� �����ϰ� �ε��մϴ�.
// * @param sim �ùķ��̼� �ν��Ͻ�
// * @return ����:1
// */
//static int simulation_setup_map(Simulation* sim) {
//    printf(C_B_WHT "--- �� ���� (1~5) ---\n" C_NRM);
//    printf("1. %s�⺻ ������%s (����)\n", C_B_GRN, C_NRM);
//    printf("2. %s������Ʈ�� 1���� ����%s\n", C_B_YEL, C_NRM);
//    printf("3. %s8 AGV + 900ĭ%s (��ŸƮ 16��6, A~H)\n", C_B_YEL, C_NRM);
//    printf("4. %s���ڵ���(1����) + ������� 4��%s (��ŸƮ 10��4, A~J)\n", C_B_YEL, C_NRM);
//    printf("5. %s���ڰ� ��%s (�߾� ������, ���� ������Ʈ, +4ĭ�� ����)\n\n", C_B_YEL, C_NRM);
//    int mid = get_integer_input("�� ��ȣ�� �����ϼ��� (1~5): ", 1, 5);
//    sim->map_id = mid;
//    grid_map_load_scenario(sim->map, sim->agent_manager, mid);
//    logger_log(sim->logger, "[%sMap%s] �� #%d �ε� �Ϸ�.", C_B_CYN, C_NRM, mid);
//    do_ms_pause(800);
//    return 1;
//}
//
///**
// * @brief ��/�˰���/���� ��带 ���ʷ� �����մϴ�.
// * @param sim �ùķ��̼� �ν��Ͻ�
// * @return ����:1, ���:0
// */
//int simulation_setup(Simulation* sim) {
//    ui_clear_screen_optimized();
//    // �� ���� �� ����
//    if (!simulation_setup_map(sim)) return 0;
//
//    // �� ��ΰ�ȹ �˰��� ����
//    printf(C_B_WHT "\n--- ��ΰ�ȹ �˰��� ���� ---\n" C_NRM);
//    printf("1. %s�⺻(WHCA* + D* Lite + WFG + CBS)%s\n", C_B_GRN, C_NRM);
//    printf("2. %sA* (�ڵ�2) - �ܼ� A* ��� �� ���� ��ȹ%s\n", C_B_YEL, C_NRM);
//    printf("3. %sD* Lite (�ڵ�3) - ������ �⺻ ������%s\n\n", C_B_YEL, C_NRM);
//    {
//        int a = get_integer_input("�˰��� ��ȣ (1~3): ", 1, 3);
//        sim->path_algo = (a == 2) ? PATHALGO_ASTAR_SIMPLE : (a == 3) ? PATHALGO_DSTAR_BASIC : PATHALGO_DEFAULT;
//        logger_log(sim->logger, "[%sAlgo%s] ����: %d", C_B_CYN, C_NRM, a);
//        // �˰��� ���� �⺻ ���� ������ stride �ڵ� ����(�ʱ� �� ��ȭ)
//        if (sim->path_algo == PATHALGO_DEFAULT) g_renderer.render_stride = 1;         // ���� �˰���: �⺻ �״��
//        else if (sim->path_algo == PATHALGO_DSTAR_BASIC) g_renderer.render_stride = 2; // ������: �ణ�� ��ŵ
//        else g_renderer.render_stride = 2;                                            // A*: ����ϰ� 2��
//        // ���� �÷��� ���ε�
//        sim->planner = planner_from_pathalgo(sim->path_algo);
//    }
//
//    printf(C_B_WHT "\n--- �ùķ��̼� ��� ���� ---\n" C_NRM);
//    printf("a. %s����� ���� �ó�����%s\n", C_YEL, C_NRM);
//    printf("b. %s�ǽð� �ùķ��̼�%s\n", C_CYN, C_NRM);
//    printf("q. %s����%s\n\n", C_RED, C_NRM);
//    char c = get_char_input("������ ���: ", "abq");
//    int ok = 0;
//    switch (c) {
//    case 'a': sim->scenario_manager->mode = MODE_CUSTOM;
//        if (simulation_setup_custom_scenario(sim))
//            ok = simulation_setup_speed(sim->scenario_manager);
//        break;
//    case 'b': sim->scenario_manager->mode = MODE_REALTIME;
//        if (simulation_setup_realtime(sim->scenario_manager))
//            ok = simulation_setup_speed(sim->scenario_manager);
//        break;
//    case 'q': return 0;
//    }
//    if (ok) ui_clear_screen_optimized();
//    return ok;
//}
//
///**
// * @brief �� ƽ�� ���� ����(����, �۾� ����/�Ҵ�, IDLE ������Ʈ Ȱ��ȭ)�� �����մϴ�.
// * @param sim �ùķ��̼� �ν��Ͻ�
// */
//static void simulation_update_state(Simulation* sim) {
//    ScenarioManager* sc = sim->scenario_manager;
//    AgentManager* am = sim->agent_manager;
//    GridMap* map = sim->map;
//    Logger* lg = sim->logger;
//
//    if (sc->mode == MODE_CUSTOM) {
//        if (sc->current_phase_index >= sc->num_phases) return;
//        DynamicPhase* ph = &sc->phases[sc->current_phase_index];
//        if (sc->tasks_completed_in_phase >= ph->task_count) {
//            logger_log(lg, "[%sPhase%s] %d�ܰ� (%s %d��) �Ϸ�!", C_B_YEL, C_NRM,
//                sc->current_phase_index + 1, ph->type_name, ph->task_count);
//            sc->current_phase_index++; sc->tasks_completed_in_phase = 0;
//            if (sc->current_phase_index < sc->num_phases) {
//                DynamicPhase* nx = &sc->phases[sc->current_phase_index];
//                logger_log(lg, "[%sPhase%s] %d�ܰ� ����: %s %d��.",
//                    C_B_YEL, C_NRM, sc->current_phase_index + 1, nx->type_name, nx->task_count);
//                do_ms_pause(1500);
//            }
//            return;
//        }
//    }
//    else if (sc->mode == MODE_REALTIME) {
//        if (sc->time_step > 0) {
//            // ��밪 ��: 100% -> ƽ�� 0.2 Ȯ�� �� ��� 5ƽ/��
//            // ���� p%�� �� ƽ�� Ȯ�� = p/500
//            int roll_park = rand() % 500;
//            int roll_exit = rand() % 500;
//
//            if (sc->park_chance > 0 && roll_park < sc->park_chance) {
//                int before = sc->task_count;
//                if (am->total_cars_parked < map->num_goals) {
//                    logger_log(lg, "[%sEvent%s] ���ο� ���� ��û.", C_B_GRN, C_NRM);
//                    add_task_to_queue(sc, TASK_PARK);
//                }
//                if (sim && sc->task_count > before) sim->requests_created_total++;
//            }
//
//            if (sc->exit_chance > 0 && roll_exit < sc->exit_chance) {
//                int before2 = sc->task_count;
//                if (am->total_cars_parked > 0) {
//                    logger_log(lg, "[%sEvent%s] ���ο� ���� ��û.", C_B_YEL, C_NRM);
//                    add_task_to_queue(sc, TASK_EXIT);
//                }
//                if (sim && sc->task_count > before2) sim->requests_created_total++;
//            }
//        }
//    }
//
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* ag = &am->agents[i];
//        if (ag->state == IDLE) {
//            if (!ag->pos) continue;
//            if (ag->total_distance_traveled >= DISTANCE_BEFORE_CHARGE) {
//                if (select_best_charge_station(ag, map, am, lg)) { ag->state = GOING_TO_CHARGE; }
//                else logger_log(lg, "[%sWarn%s] Agent %c ���� �ʿ��ϳ� ������ �����.", C_YEL, C_NRM, ag->symbol);
//                continue;
//            }
//            if (sc->mode == MODE_CUSTOM) {
//                if (sc->current_phase_index >= sc->num_phases) continue;
//                DynamicPhase* ph = &sc->phases[sc->current_phase_index];
//                if (ph->type == PARK_PHASE) {
//                    int active = 0; for (int j = 0; j < MAX_AGENTS; j++) {
//                        AgentState s = am->agents[j].state;
//                        if (s == GOING_TO_PARK || s == RETURNING_HOME_EMPTY) active++;
//                    }
//                    if ((sc->tasks_completed_in_phase + active) < ph->task_count && am->total_cars_parked < map->num_goals) {
//                        g_agent_ops.beginTaskPark(ag, sc, lg);
//                    }
//                }
//                else {
//                    int active = 0; for (int j = 0; j < MAX_AGENTS; j++) {
//                        AgentState s = am->agents[j].state;
//                        if (s == GOING_TO_COLLECT || s == RETURNING_WITH_CAR) active++;
//                    }
//                    if ((sc->tasks_completed_in_phase + active) < ph->task_count && am->total_cars_parked > 0) {
//                        g_agent_ops.beginTaskExit(ag, sc, lg);
//                    }
//                }
//            }
//            else if (sc->mode == MODE_REALTIME && sc->task_count > 0) {
//                int lot_full = (am->total_cars_parked >= map->num_goals);
//                TaskNode* cur = sc->task_queue_head; TaskNode* prev = NULL;
//                while (cur) {
//                    int can = FALSE;
//                    if (lot_full) {
//                        if (cur->type == TASK_EXIT && am->total_cars_parked > 0) can = TRUE;
//                    }
//                    else {
//                        if (cur->type == TASK_PARK) can = TRUE;
//                        else if (cur->type == TASK_EXIT && am->total_cars_parked > 0) can = TRUE;
//                    }
//                    if (can) {
//                        if (cur->type == TASK_PARK) { g_agent_ops.beginTaskPark(ag, sc, lg); }
//                        else { g_agent_ops.beginTaskExit(ag, sc, lg); }
//                        if (prev == NULL) sc->task_queue_head = cur->next; else prev->next = cur->next;
//                        if (cur == sc->task_queue_tail) sc->task_queue_tail = prev;
//                        free(cur); sc->task_count--; break;
//                    }
//                    prev = cur; cur = cur->next;
//                }
//            }
//        }
//    }
//}
//
//// --- One-step executor: encapsulates a single simulation tick without input/pause handling ---
///**
// * @brief �Է�/�Ͻ����� ó�� ���� ���� �� ƽ�� �����մϴ�.
// *        (����/���°��� �� ��ȹ �� �̵����� �� �������� �� ��Ʈ��/����)
// * @param sim �ùķ��̼� �ν��Ͻ�
// * @param is_paused �Ͻ����� �÷���(ǥ�ÿ�)
// */
//static void simulation_execute_one_step(Simulation* sim, int is_paused) {
//    ScenarioManager* sc = sim->scenario_manager;
//    int phase_idx_for_step = sc->current_phase_index;
//    int step_label = sc->time_step + 1;
//    int is_custom_mode = (sc->mode == MODE_CUSTOM);
//    int phase_active = (is_custom_mode && phase_idx_for_step >= 0 && phase_idx_for_step < sc->num_phases);
//    int cleanup_region = (is_custom_mode && phase_idx_for_step >= sc->num_phases);
//
//    clock_t step_start_cpu = clock();
//
//    agent_manager_update_charge_state(sim->agent_manager, sim->map, sim->logger);
//    simulation_update_state(sim);
//
//    Node* next_pos[MAX_AGENTS];
//    Node* prev_pos[MAX_AGENTS];
//    for (int i = 0; i < MAX_AGENTS; i++) prev_pos[i] = sim->agent_manager->agents[i].pos;
//
//    // ��ȹ �ܰ� ĸ��ȭ
//    simulation_plan_step(sim, next_pos);
//
//    // ȸ�� ���(TURN_90_WAIT) ����: ��� �˰��� ���� ó��
//    {
//        AgentManager* am = sim->agent_manager;
//        for (int i = 0; i < MAX_AGENTS; i++) {
//            Agent* ag = &am->agents[i];
//            if (ag->state == CHARGING) continue;
//            Node* current = ag->pos;
//            if (!current || !next_pos[i]) continue;
//            if (ag->rotation_wait > 0) {
//                next_pos[i] = current;
//                ag->rotation_wait--;
//                continue;
//            }
//            Node* adjusted = current;
//            agent_apply_rotation_and_step(ag, current, next_pos[i], &adjusted);
//            next_pos[i] = adjusted;
//        }
//
//        // ȸ�� ��� ���� ������Ʈ ĭ ���� ����(��� ����)
//        for (int i = 0; i < MAX_AGENTS; i++) {
//            Agent* blocker = &am->agents[i];
//            if (!blocker->pos) continue;
//            if (blocker->rotation_wait <= 0) continue;
//            Node* blocked_cell = blocker->pos;
//            for (int j = 0; j < MAX_AGENTS; j++) {
//                if (j == i) continue;
//                Agent* mover = &am->agents[j];
//                if (!mover->pos || !next_pos[j]) continue;
//                if (next_pos[j] == blocked_cell && mover->pos != blocked_cell) {
//                    next_pos[j] = mover->pos; // ��� ó��
//                }
//            }
//        }
//
//        // �߰� ������ġ: �̹� ƽ�� "����"�ϴ� ������Ʈ�� ���� ĭ���� ���� ����
//        // (ȸ�� ��� �ܿ��� �۾� ��� ������ �����ϴ� ��� ����)
//        for (int i = 0; i < MAX_AGENTS; i++) {
//            Agent* stopper = &am->agents[i];
//            if (!stopper->pos) continue;
//            if (!next_pos[i]) continue;
//            if (next_pos[i] != stopper->pos) continue; // �����̴� ������Ʈ�� ����
//            Node* blocked_cell = stopper->pos;
//            for (int j = 0; j < MAX_AGENTS; j++) {
//                if (j == i) continue;
//                Agent* mover = &am->agents[j];
//                if (!mover->pos || !next_pos[j]) continue;
//                if (next_pos[j] == blocked_cell && mover->pos != blocked_cell) {
//                    next_pos[j] = mover->pos; // ��� ó��
//                }
//            }
//        }
//    }
//
//    // ȸ�� ���� �� ���� ���� ���� ����, ���� �ִ� �浹 ������ �� �� �� ����(�� �˰��� ����)
//    {
//        int order[MAX_AGENTS];
//        sort_agents_by_priority(sim->agent_manager, order);
//        resolve_conflicts_by_order(sim->agent_manager, order, next_pos);
//    }
//
//    int moved_this_step = apply_moves_and_update_stuck(sim, next_pos, prev_pos);
//
//    unsigned long long prev_completed_tasks = sim->tasks_completed_total;
//    agent_manager_update_state_after_move(sim->agent_manager, sim->scenario_manager, sim->map, sim->logger, sim);
//    if (sim->tasks_completed_total != prev_completed_tasks) {
//        sim->last_task_completion_step = step_label;
//    }
//
//    clock_t step_end_cpu = clock();
//    double step_time_ms = ((double)(step_end_cpu - step_start_cpu) * 1000.0) / CLOCKS_PER_SEC;
//    sim->last_step_cpu_time_ms = step_time_ms;
//    sim->total_cpu_time_ms += step_time_ms;
//    if (step_time_ms > sim->max_step_cpu_time_ms) {
//        sim->max_step_cpu_time_ms = step_time_ms;
//    }
//    if (is_custom_mode) {
//        if (phase_active) {
//            int idx = phase_idx_for_step;
//            if (idx >= 0 && idx < MAX_PHASES) {
//                if (sim->phase_step_counts[idx] == 0) {
//                    sim->phase_first_step[idx] = step_label;
//                }
//                sim->phase_last_step[idx] = step_label;
//                sim->phase_step_counts[idx]++;
//                sim->phase_cpu_time_ms[idx] += step_time_ms;
//            }
//        }
//        else if (cleanup_region) {
//            if (sim->post_phase_step_count == 0) {
//                sim->post_phase_first_step = step_label;
//            }
//            sim->post_phase_last_step = step_label;
//            sim->post_phase_step_count++;
//            sim->post_phase_cpu_time_ms += step_time_ms;
//            // ��� �ܰ谡 ���� �� ���� ������ ������ �ܿ� ������Ʈ�� ���������� ���� ����ȭ�� ���� �ؼ�
//            if (sim->post_phase_step_count >= CLEANUP_FORCE_IDLE_AFTER_STEPS) {
//                force_idle_cleanup(sim->agent_manager, sim, sim->logger);
//            }
//        }
//    }
//
//    update_deadlock_counter(sim, moved_this_step, is_custom_mode);
//
//    accumulate_wait_ticks_if_realtime(sim);
//
//    // �˰��� �ܰ� ���ø� ����
//    simulation_collect_memory_sample_algo(sim);
//    // ��ü ���μ��� �޸� ��뷮 ���� ����
//    simulation_collect_memory_sample(sim);
//    sim->total_executed_steps = step_label;
//
//    // ������ ���� (������ ��ŵ�� renderer ���ο��� ó����)
//    sim->renderer.vtbl.draw_frame(sim, is_paused);
//}
//
//// ��� ������Ʈ�� ������ IDLE ���·� ��ȯ�Ͽ� ���� �ܰ迡���� ������ �ؼ��Ѵ�
//static void force_idle_cleanup(AgentManager* am, Simulation* sim, Logger* lg) {
//    if (!am) return;
//    int changed = 0;
//    for (int i = 0; i < MAX_AGENTS; i++) {
//        Agent* ag = &am->agents[i];
//        if (!ag->pos) continue;
//        if (ag->state == IDLE) continue;
//        if (ag->goal) { ag->goal->reserved_by_agent = -1; ag->goal = NULL; }
//        if (ag->pf) { pathfinder_destroy(ag->pf); ag->pf = NULL; }
//        ag->rotation_wait = 0;
//        ag->stuck_steps = 0;
//        ag->action_timer = 0;
//        ag->state = IDLE;
//        changed = 1;
//    }
//    if (changed && lg) {
//        logger_log(lg, "[%sCleanup%s] ��� ������Ʈ�� ���� IDLE�� ��ȯ�Ͽ� ������ �ؼ��߽��ϴ�.", C_B_CYN, C_NRM);
//    }
//}
///**
// * @brief �ùķ��̼� ���� ����(Ŀ����: ��� �ܰ� �Ϸ�, �ǽð�: �ð� ����)�� Ȯ���մϴ�.
// * @param sim �ùķ��̼� �ν��Ͻ�
// * @return ����:TRUE, ���:FALSE
// */
//static int simulation_is_complete(const Simulation* sim) {
//    const ScenarioManager* sc = sim->scenario_manager;
//    const AgentManager* am = sim->agent_manager;
//    if (sc->mode == MODE_CUSTOM && sc->current_phase_index >= sc->num_phases) {
//        for (int i = 0; i < MAX_AGENTS; i++) if (am->agents[i].state != IDLE) return FALSE;
//        printf(C_B_GRN "\n��� �ܰ� �Ϸ�! �����մϴ�.\n" C_NRM); return TRUE;
//    }
//    if (sc->mode == MODE_REALTIME && sc->time_step >= REALTIME_MODE_TIMELIMIT) {
//        printf(C_B_GRN "\n�ð� ���� ����! �����մϴ�.\n" C_NRM); return TRUE;
//    }
//    return FALSE;
//}
//
//// *** MODIFIED *** �ǽð� ���� �� ������ ���� ���� ����
///**
// * @brief ���� ����: �񵿱� �Է�(P/S/��/[]/F/C/Q) ó���� �� ƽ ������ �ݺ��մϴ�.
// * @param sim �ùķ��̼� �ν��Ͻ�
// */
//void simulation_run(Simulation* sim) {
//    ControlState cs; ControlState_init(&cs);
//
//    simulation_reset_runtime_stats(sim);
//
//    // �ùķ��̼� ���� ��, �ʱ� ���¸� �� �� �׸�
//    sim->renderer.vtbl.draw_frame(sim, cs.is_paused);
//
//    while (!cs.quit_flag) {
//        // --- �Է� ó�� ---
//        cs.last_key = check_for_input();
//        if (cs.last_key) {
//            ui_handle_control_key(sim, cs.last_key, &cs.is_paused, &cs.quit_flag);
//            sim->renderer.vtbl.draw_frame(sim, cs.is_paused);
//            if (cs.quit_flag) continue;
//        }
//
//        // --- �Ͻ����� ���� ---
//        // �Ͻ����� �����̰�, 's'Ű�� �Էµ��� �ʾҴٸ� �ùķ��̼� ������ �ǳʶ�
//        if (cs.is_paused && tolower(cs.last_key) != 's') {
//            sleep_ms(PAUSE_POLL_INTERVAL_MS);
//            continue;
//        }
//
//        // --- �� ���� ���� ---
//        simulation_execute_one_step(sim, cs.is_paused);
//
//        if (simulation_is_complete(sim)) {
//            break;
//        }
//
//        maybe_report_realtime_dashboard(sim);
//
//        if (sim->scenario_manager->simulation_speed > 0) sleep_ms(sim->scenario_manager->simulation_speed);
//    }
//}
//
///**
// * @brief ���� ���(����/ó����/�̵��Ÿ�/�޸�/�ܰ躰 ���)�� ����մϴ�.
// * @param sim �ùķ��̼� �ν��Ͻ�
// */
//void simulation_print_performance_summary(const Simulation* sim) {
//    const ScenarioManager* sc = sim->scenario_manager;
//    const AgentManager* am = sim->agent_manager;
//    const int recorded_steps = (sim->total_executed_steps > 0) ? sim->total_executed_steps : (sc ? sc->time_step : 0);
//    const double avg_cpu_ms = (recorded_steps > 0) ? (sim->total_cpu_time_ms / (double)recorded_steps) : 0.0;
//    const double avg_plan_ms = (recorded_steps > 0) ? (sim->total_planning_time_ms / (double)recorded_steps) : 0.0;
//    const double throughput = (recorded_steps > 0) ? ((double)sim->tasks_completed_total / (double)recorded_steps) : 0.0;
//    const double avg_memory_kb = (sim->memory_samples > 0) ? (sim->memory_usage_sum_kb / (double)sim->memory_samples) : 0.0;
//
//    const char* mode_label = "Uninitialized";
//    if (sc) {
//        switch (sc->mode) {
//        case MODE_CUSTOM: mode_label = "Custom"; break;
//        case MODE_REALTIME: mode_label = "Real-Time"; break;
//        default: mode_label = "Uninitialized"; break;
//        }
//    }
//
//    printf("\n============================================\n");
//    printf("          Simulation Result Report\n");
//    printf("============================================\n");
//    printf(" Mode                                : %s\n", mode_label);
//    printf(" Map ID                              : %d\n", sim->map_id);
//    {
//        const char* algo = "Default (WHCA* + D* Lite + WFG + CBS)";
//        if (sim->path_algo == PATHALGO_ASTAR_SIMPLE) algo = "A* (�ܼ�)";
//        else if (sim->path_algo == PATHALGO_DSTAR_BASIC) algo = "D* Lite (�⺻)";
//        printf(" Path Planning Algorithm             : %s\n", algo);
//    }
//    printf(" Total Physical Time Steps           : %d\n", recorded_steps);
//    {
//        int active_agents = 0;
//        if (am) {
//            for (int i = 0; i < MAX_AGENTS; i++) if (am->agents[i].pos) active_agents++;
//        }
//        printf(" Operating AGVs                     : %d\n", active_agents);
//    }
//
//    printf(" Tasks Completed (total)             : %llu\n", sim->tasks_completed_total);
//    printf(" Throughput [task / total physical time] : %.4f\n", throughput);
//    printf(" Total Movement Cost (cells)         : %.2f\n", sim->total_movement_cost);
//
//    printf(" Requests Created (total)            : %llu\n", sim->requests_created_total);
//    printf(" Request Wait Ticks (sum)            : %llu\n", sim->request_wait_ticks_sum);
//    printf(" Process Memory Usage Sum            : %.2f KB\n", sim->memory_usage_sum_kb);
//    printf(" Process Memory Usage Average        : %.2f KB\n", avg_memory_kb);
//    printf(" Process Memory Usage Peak           : %.2f KB\n", sim->memory_usage_peak_kb);
//    printf(" Remaining Parked Vehicles           : %d\n", am ? am->total_cars_parked : 0);
//
//    if (sc && sc->mode == MODE_CUSTOM) {
//        printf("\n -- Custom Scenario Breakdown --\n");
//        for (int i = 0; i < sc->num_phases; i++) {
//            const DynamicPhase* ph = &sc->phases[i];
//            const int planned = ph->task_count;
//            const int completed = sim->phase_completed_tasks[i];
//            const int step_count = sim->phase_step_counts[i];
//            printf(" Phase %d (%s)\n", i + 1, ph->type_name);
//            printf("   Planned Tasks           : %d\n", planned);
//            printf("   Completed Tasks         : %d\n", completed);
//            if (step_count > 0) {
//                printf("   Step Span               : %d step(s)", step_count);
//                if (sim->phase_first_step[i] >= 0 && sim->phase_last_step[i] >= 0) {
//                    printf(" [#%d -> #%d]\n", sim->phase_first_step[i], sim->phase_last_step[i]);
//                }
//                else {
//                    printf("\n");
//                }
//                const double phase_avg_cpu = sim->phase_cpu_time_ms[i] / (double)step_count;
//                printf("   CPU Time                : %.2f ms (avg %.4f ms/step)\n",
//                    sim->phase_cpu_time_ms[i], phase_avg_cpu);
//            }
//            else {
//                printf("   Step Span               : N/A\n");
//            }
//            if (completed < planned) {
//                printf("   Remaining Tasks         : %d\n", planned - completed);
//            }
//        }
//    }
//    else if (sc && sc->mode == MODE_REALTIME) {
//        printf("\n -- Custom Scenario Breakdown --\n");
//        // Real-Time ���: ���� ���� �������� ���� ���� ����
//        printf(" Phase 1 (%s)\n", "Real-Time");
//        printf("   Planned Tasks           : %d\n", (int)sim->tasks_completed_total);
//        printf("   Completed Tasks         : %d\n", (int)sim->tasks_completed_total);
//        if (recorded_steps > 0) {
//            printf("   Step Span               : %d step(s) [#%d -> #%d]\n", recorded_steps, 1, recorded_steps);
//            const double phase_avg_cpu = sim->total_cpu_time_ms / (double)recorded_steps;
//            printf("   CPU Time                : %.2f ms (avg %.4f ms/step)\n", sim->total_cpu_time_ms, phase_avg_cpu);
//        }
//        else {
//            printf("   Step Span               : N/A\n");
//            printf("   CPU Time                : 0.00 ms (avg 0.0000 ms/step)\n");
//        }
//    }
//
//    printf("============================================\n");
//}
//
//// =============================================================================
//// ���� 11: �ùķ��̼� �����ֱ� (����/�ı�) �� ���� ������
//// =============================================================================
//Simulation* simulation_create() {
//    Simulation* s = (Simulation*)calloc(1, sizeof(Simulation)); if (!s) { perror("Simulation"); exit(1); }
//    s->agent_manager = agent_manager_create();
//    s->map = grid_map_create(s->agent_manager); // default map #1
//    s->scenario_manager = scenario_manager_create();
//    s->logger = logger_create();
//    s->map_id = 1; // ���� �� = 1��
//    s->path_algo = PATHALGO_DEFAULT;
//    s->planner = planner_from_pathalgo(s->path_algo);
//    s->renderer = renderer_create_facade();
//    g_metrics.whca_h = g_whca_horizon;
//    GlobalConfig_init(&g_config);
//    metrics_subscribe(simulation_metrics_observer, s);
//    return s;
//}
//void simulation_destroy(Simulation* s) {
//    if (s) {
//        grid_map_destroy(s->map);
//        agent_manager_destroy(s->agent_manager);
//        scenario_manager_destroy(s->scenario_manager);
//        logger_destroy(s->logger);
//        free(s);
//    }
//}
//int main() {
//    srand((unsigned int)time(NULL));
//    system_enable_virtual_terminal();
//    ensure_console_width(180);
//
//
//    Simulation* sim = simulation_create();
//    if (!sim) return 1;
//
//    if (simulation_setup(sim)) {
//        ui_enter_alt_screen();     // ALT ��ũ�� ����
//        simulation_run(sim);       // ������ �����ص� ��ũ�ѹ� ���� ����
//        ui_leave_alt_screen();     // ALT ��ũ�� ����(���� ȭ�� ����)
//        simulation_print_performance_summary(sim);
//        printf("\n����� Ȯ���Ϸ��� �ƹ� Ű�� ��������...\n");
//        (void)_getch();
//    }
//    else {
//        printf("\n�ùķ��̼��� ��ҵǾ����ϴ�. �����մϴ�.\n");
//    }
//
//
//    simulation_destroy(sim);
//    return 0;
//}