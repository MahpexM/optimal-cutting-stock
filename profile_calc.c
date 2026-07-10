/*
 * ============================================================================
 *  Задача о раскрое (1D Cutting Stock Problem)
 * ============================================================================
 * 
 *  Дано:
 *    M заготовок с длинами K1..KM  (то, что есть / можно купить)
 *    N деталей   с длинами L1..LN  (то, что нужно изготовить)
 *    h — технологический запас (kerf), теряемый на каждом резе между
 *        соседними деталями внутри одной заготовки.
 *
 *  Критерий оптимальности — ЛЕКСИКОГРАФИЧЕСКИЙ (не свёртка в одно число!):
 *    1) минимизировать суммарный отход  sum(R_k)  по ЗАДЕЙСТВОВАННЫМ
 *       заготовкам (заготовки, которые вообще не резали, — это не отход,
 *       а просто неиспользованный остаток комплекта, о нём сообщаем отдельно);
 *    2) при равенстве отхода — минимизировать суммарное число резов.
 *
 *  ВАЖНОЕ ДОПУЩЕНИЕ про kerf (h):
 *    Если на заготовку K_i назначено c деталей суммарной длиной S, то между
 *    ними делается (c-1) резов, и это единственные резы, которые мы
 *    учитываем в остатке:
 *        R_i = K_i - S - h*(c-1)
 *    Отдельный (последний) рез, которым можно было бы физически отделить
 *    остаток R_i от последней детали, НЕ учитывается (остаток считается
 *    "виртуально прилегающим" к последней детали). Если в вашем производстве
 *    остаток всегда физически отрезается отдельным резом — замените в коде
 *    h*(c-1) на h*c везде, где помечено комментарием "KERF-ASSUMPTION".
 *
 *  Методы решения (можно сравнить все три на одном входе):
 *    1. build_reference_plan_ffd()  — First-Fit Decreasing, "опорный план"
 *                                      (базовое допустимое решение, быстро,
 *                                      не обязательно хорошее).
 *    2. build_plan_bfd()            — Best-Fit Decreasing, эвристика получше.
 *    3. local_search_improve()      — локальный поиск (обмен деталями между
 *                                      заготовками), дожимает BFD-план.
 *    4. solve_exact_bnb()           — точный перебор с отсечениями,
 *                                      только для небольших M,N (см. порог
 *                                      EXACT_SIZE_LIMIT), гарантирует
 *                                      лексикографический оптимум.
 *
 *  Компиляция:
 *	  gcc profile_calc.c -o cutting_stock
 *
 *  Формат вызова:
 *    ./cutting_stock [-h H] M K1 K2 ... KM N L1 L2 ... LN
 *
 *  Пример:
 *    ./cutting_stock -h 3 3 4701 8808 456 4 3205 740 740 740
 *
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>

/* Порог суммарной размерности (M+N), выше которого точный перебор
 * не запускается автоматически (комбинаторный взрыв). Подбирается
 * эмпирически под приемлемое время работы; можно поднять для более
 * мощной машины / более терпеливого пользователя. */
#define EXACT_SIZE_LIMIT 18

/* ---------------------------------------------------------------------- */
/*  Структуры данных                                                       */
/* ---------------------------------------------------------------------- */

typedef struct {
    uint32_t *K;   /* длины заготовок, в исходном порядке ввода   */
    size_t    M;
    uint32_t *L;   /* длины деталей,   в исходном порядке ввода   */
    size_t    N;
    uint32_t  h;   /* технологический запас (kerf), по умолчанию 0 */
} input_t;

/* Раскрой одной заготовки: список исходных индексов деталей (в L),
 * которые из неё вырезаны, в порядке назначения. */
typedef struct {
    size_t  *piece_idx;   /* индексы в input_t.L                     */
    size_t   count;       /* сколько деталей вырезано из этой заготовки */
    size_t   cap;         /* выделенная ёмкость piece_idx (для realloc) */
    uint32_t consumed;    /* сумма длин деталей (без kerf)            */
    uint32_t remainder;   /* R_k = K_k - consumed - h*(count-1)       */
} stock_cut_t;

typedef struct {
    stock_cut_t *stocks;      /* размер M, индексация совпадает с input_t.K */
    size_t       M;
    uint64_t     total_waste; /* sum(remainder) по ЗАДЕЙСТВОВАННЫМ заготовкам */
    uint64_t     total_cuts;  /* sum(count-1) по задействованным заготовкам   */
    int          all_assigned;/* 1, если все N деталей успешно размещены      */
    const char  *method_name;
} plan_t;

/* ---------------------------------------------------------------------- */
/*  Парсинг аргументов командной строки                                    */
/* ---------------------------------------------------------------------- */

static int parse_u32(const char *s, uint32_t *out)
{
    if (!s || !*s) return -1;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 10);
    if (end == s || *end != '\0' || v > UINT32_MAX) return -1;
    *out = (uint32_t)v;
    return 0;
}

/*
 * @description Разбор argv в формат [-h H] M K1..KM N L1..LN
 * @param argc, argv Стандартные параметры main()
 * @param in Указатель на заполняемую структуру входных данных
 * @returns 0 при успехе, -1 при ошибке разбора (некорректный формат)
 */
static int parse_args(int argc, char **argv, input_t *in)
{
    memset(in, 0, sizeof(*in));

    /* 1) вычленяем необязательный флаг -h H в любом месте */
    char **filtered = malloc((size_t)argc * sizeof(char *));
    if (!filtered) { fprintf(stderr, "Ошибка: не удалось выделить память\n"); return -1; }
    int fcount = 0;
    in->h = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0) {
            if (i + 1 >= argc || parse_u32(argv[i + 1], &in->h) != 0) {
                fprintf(stderr, "Ошибка: после -h ожидается целое число (запас на рез)\n");
                free(filtered);
                return -1;
            }
            i++; /* пропускаем значение h */
            continue;
        }
        filtered[fcount++] = argv[i];
    }

    /* 2) позиционный разбор: M K1..KM N L1..LN */
    if (fcount < 2) {
        fprintf(stderr, "Ошибка: недостаточно аргументов (нужно как минимум M и N)\n");
        free(filtered);
        return -1;
    }

    size_t pos = 0;
    uint32_t M_val;
    if (parse_u32(filtered[pos++], &M_val) != 0 || M_val < 1) {
        fprintf(stderr, "Ошибка: M должно быть натуральным числом\n");
        free(filtered);
        return -1;
    }
    in->M = M_val;

    if ((size_t)fcount < pos + in->M + 1) {
        fprintf(stderr, "Ошибка: не хватает аргументов для K1..KM и N\n");
        free(filtered);
        return -1;
    }

    in->K = malloc(in->M * sizeof(uint32_t));
    if (!in->K) { fprintf(stderr, "Ошибка выделения памяти под K\n"); free(filtered); return -1; }
    for (size_t i = 0; i < in->M; i++) {
        if (parse_u32(filtered[pos++], &in->K[i]) != 0) {
            fprintf(stderr, "Ошибка: K%zu не является натуральным числом\n", i + 1);
            free(filtered); free(in->K); in->K = NULL;
            return -1;
        }
    }

    uint32_t N_val;
    if (parse_u32(filtered[pos++], &N_val) != 0 || N_val < 1) {
        fprintf(stderr, "Ошибка: N должно быть натуральным числом\n");
        free(filtered); free(in->K); in->K = NULL;
        return -1;
    }
    in->N = N_val;

    if ((size_t)fcount < pos + in->N) {
        fprintf(stderr, "Ошибка: не хватает аргументов для L1..LN\n");
        free(filtered); free(in->K); in->K = NULL;
        return -1;
    }

    in->L = malloc(in->N * sizeof(uint32_t));
    if (!in->L) {
        fprintf(stderr, "Ошибка выделения памяти под L\n");
        free(filtered); free(in->K); in->K = NULL;
        return -1;
    }
    for (size_t j = 0; j < in->N; j++) {
        if (parse_u32(filtered[pos++], &in->L[j]) != 0) {
            fprintf(stderr, "Ошибка: L%zu не является натуральным числом\n", j + 1);
            free(filtered); free(in->K); free(in->L);
            in->K = NULL; in->L = NULL;
            return -1;
        }
    }

    free(filtered);
    return 0;
}

static void free_input(input_t *in)
{
    free(in->K); in->K = NULL;
    free(in->L); in->L = NULL;
}

/* ---------------------------------------------------------------------- */
/*  Вспомогательное: сортировка "значение + исходный индекс" по убыванию   */
/* ---------------------------------------------------------------------- */

typedef struct { uint32_t value; size_t orig_idx; } indexed_u32_t;

static int cmp_indexed_desc(const void *a, const void *b)
{
    const indexed_u32_t *ia = a, *ib = b;
    if (ia->value != ib->value) return (ia->value < ib->value) ? 1 : -1;
    /* при равенстве значений — по возрастанию исходного индекса,
     * чтобы порядок вывода был детерминированным между запусками */
    return (ia->orig_idx > ib->orig_idx) ? 1 : -1;
}

static indexed_u32_t *make_sorted_desc(const uint32_t *arr, size_t n)
{
    indexed_u32_t *out = malloc(n * sizeof(indexed_u32_t));
    if (!out) return NULL;
    for (size_t i = 0; i < n; i++) { out[i].value = arr[i]; out[i].orig_idx = i; }
    qsort(out, n, sizeof(indexed_u32_t), cmp_indexed_desc);
    return out;
}

/* ---------------------------------------------------------------------- */
/*  Проверка принципиальной выполнимости                                   */
/* ---------------------------------------------------------------------- */

/*
 * @description Проверяет, в принципе ли достижим раскрой без учёта
 *              конкретной комбинаторики (необходимые, но не достаточные
 *              условия): максимальная деталь должна помещаться хотя бы
 *              в одну заготовку, суммарная длина заготовок не меньше
 *              суммарной длины деталей.
 * @returns 0 — похоже на выполнимое, -1 — точно невыполнимое
 */
static int check_feasibility(const input_t *in)
{
    uint32_t max_L = 0, max_K = 0;
    uint64_t sum_K = 0, sum_L = 0;

    for (size_t i = 0; i < in->M; i++) { sum_K += in->K[i]; if (in->K[i] > max_K) max_K = in->K[i]; }
    for (size_t j = 0; j < in->N; j++) { sum_L += in->L[j]; if (in->L[j] > max_L) max_L = in->L[j]; }

    if (max_L > max_K) {
        fprintf(stderr,
            "MaxElementLengthError: деталь длиной %u мм не помещается ни в одну "
            "заготовку (максимальная заготовка — %u мм)\n", max_L, max_K);
        return -1;
    }

    /* Грубая необходимая проверка без учёта h (реальная нехватка с учётом
     * kerf может проявиться позже, во время построения плана — тогда
     * all_assigned будет 0, и это тоже обрабатывается как ошибка). */
    if (sum_K < sum_L) {
        fprintf(stderr,
            "Ошибка: суммарная длина заготовок (%llu мм) меньше суммарной "
            "длины требуемых деталей (%llu мм) — материала не хватит ни при "
            "каком раскрое\n", (unsigned long long)sum_K, (unsigned long long)sum_L);
        return -1;
    }

    return 0;
}

/* ---------------------------------------------------------------------- */
/*  Работа с plan_t                                                         */
/* ---------------------------------------------------------------------- */

static plan_t *plan_alloc(size_t M, const char *method_name)
{
    plan_t *p = calloc(1, sizeof(plan_t));
    if (!p) return NULL;
    p->stocks = calloc(M, sizeof(stock_cut_t));
    if (!p->stocks) { free(p); return NULL; }
    p->M = M;
    p->method_name = method_name;
    p->all_assigned = 1;
    return p;
}

static void free_plan(plan_t *p)
{
    if (!p) return;
    for (size_t i = 0; i < p->M; i++) free(p->stocks[i].piece_idx);
    free(p->stocks);
    free(p);
}

/* Добавляет деталь orig_piece_idx (индекс в input_t.L) на заготовку stock_i.
 * Ёмкость заготовки НЕ проверяется здесь — проверка должна быть сделана
 * заранее вызывающим кодом (см. fits()). */
static void plan_assign(plan_t *p, const input_t *in, size_t stock_i, size_t orig_piece_idx)
{
    stock_cut_t *sc = &p->stocks[stock_i];
    if (sc->count == sc->cap) {
        size_t new_cap = sc->cap ? sc->cap * 2 : 4;
        size_t *tmp = realloc(sc->piece_idx, new_cap * sizeof(size_t));
        if (!tmp) { fprintf(stderr, "Ошибка выделения памяти при назначении детали\n"); exit(EXIT_FAILURE); }
        sc->piece_idx = tmp;
        sc->cap = new_cap;
    }
    sc->piece_idx[sc->count++] = orig_piece_idx;
    sc->consumed += in->L[orig_piece_idx];
}

/* Пересчитывает remainder, total_waste, total_cuts по актуальному
 * состоянию piece_idx/consumed/count всех заготовок плана. */
static void plan_finalize(plan_t *p, const input_t *in)
{
    p->total_waste = 0;
    p->total_cuts = 0;
    for (size_t i = 0; i < p->M; i++) {
        stock_cut_t *sc = &p->stocks[i];
        if (sc->count == 0) {
            sc->remainder = in->K[i];
            continue;
        }
        /* KERF-ASSUMPTION: учитываем только (count-1) резов между деталями */
        uint64_t kerf_used = (uint64_t)in->h * (sc->count - 1);
        uint64_t used = (uint64_t)sc->consumed + kerf_used;
        if (used > in->K[i]) {
            /* не должно происходить при корректной проверке fits(),
             * но подстрахуемся от переполнения "снизу" в uint32_t */
            sc->remainder = 0;
        } else {
            sc->remainder = (uint32_t)(in->K[i] - used);
        }
        p->total_waste += sc->remainder;
        p->total_cuts  += (sc->count - 1);
    }
}

/* Ёмкость заготовки stock_i, доступная для СЛЕДУЮЩЕЙ детали с учётом
 * резервирования kerf под предстоящий рез (см. комментарий в шапке файла). */
static uint32_t remaining_capacity(const plan_t *p, const input_t *in, size_t stock_i)
{
    const stock_cut_t *sc = &p->stocks[stock_i];
    if (sc->count == 0) return in->K[stock_i];
    uint64_t reserved = (uint64_t)sc->consumed + (uint64_t)in->h * sc->count;
    if (reserved >= in->K[stock_i]) return 0;
    return (uint32_t)(in->K[stock_i] - reserved);
}

static int fits(const plan_t *p, const input_t *in, size_t stock_i, uint32_t piece_len)
{
    return piece_len <= remaining_capacity(p, in, stock_i);
}

/* Сравнение планов: 1 — a лучше b, -1 — b лучше a, 0 — равнозначны.
 * Невыполненный план (all_assigned == 0) всегда хуже выполненного. */
static int compare_plans(const plan_t *a, const plan_t *b)
{
    if (a->all_assigned != b->all_assigned)
        return a->all_assigned ? 1 : -1;
    if (!a->all_assigned) return 0; /* оба невыполнимы — не сравниваем детальнее */
    if (a->total_waste != b->total_waste)
        return (a->total_waste < b->total_waste) ? 1 : -1;
    if (a->total_cuts != b->total_cuts)
        return (a->total_cuts < b->total_cuts) ? 1 : -1;
    return 0;
}

/* ---------------------------------------------------------------------- */
/*  Метод 1: опорный план — First-Fit Decreasing                           */
/* ---------------------------------------------------------------------- */

/*
 * @description Строит базовый допустимый план методом First-Fit Decreasing:
 *              детали перебираются по убыванию длины, каждая помещается в
 *              ПЕРВУЮ по порядку ввода заготовку, куда она физически влезает.
 *              Быстрый (O(N*M)), но не оптимизирует отход — это "опорная"
 *              точка отсчёта для сравнения с более сильными методами.
 */
static plan_t *build_reference_plan_ffd(const input_t *in)
{
    plan_t *p = plan_alloc(in->M, "FFD (опорный план)");
    if (!p) return NULL;

    indexed_u32_t *L_sorted = make_sorted_desc(in->L, in->N);
    if (!L_sorted) { free_plan(p); return NULL; }

    for (size_t j = 0; j < in->N; j++) {
        uint32_t piece_len = L_sorted[j].value;
        size_t   piece_idx = L_sorted[j].orig_idx;
        int placed = 0;
        for (size_t i = 0; i < in->M; i++) {
            if (fits(p, in, i, piece_len)) {
                plan_assign(p, in, i, piece_idx);
                placed = 1;
                break;
            }
        }
        if (!placed) p->all_assigned = 0;
    }

    free(L_sorted);
    plan_finalize(p, in);
    return p;
}

/* ---------------------------------------------------------------------- */
/*  Метод 2: Best-Fit Decreasing                                           */
/* ---------------------------------------------------------------------- */

/*
 * @description Строит план методом Best-Fit Decreasing: детали перебираются
 *              по убыванию длины, каждая помещается в заготовку с НАИМЕНЬШЕЙ
 *              достаточной оставшейся ёмкостью (минимизирует локальный
 *              "проигрыш" на каждом шаге). На практике даёт заметно меньший
 *              суммарный отход, чем FFD, при той же сложности O(N*M).
 */
static plan_t *build_plan_bfd(const input_t *in)
{
    plan_t *p = plan_alloc(in->M, "BFD (эвристика)");
    if (!p) return NULL;

    indexed_u32_t *L_sorted = make_sorted_desc(in->L, in->N);
    if (!L_sorted) { free_plan(p); return NULL; }

    for (size_t j = 0; j < in->N; j++) {
        uint32_t piece_len = L_sorted[j].value;
        size_t   piece_idx = L_sorted[j].orig_idx;

        size_t   best_i = SIZE_MAX;
        uint32_t best_leftover = UINT32_MAX;

        for (size_t i = 0; i < in->M; i++) {
            uint32_t cap = remaining_capacity(p, in, i);
            if (piece_len > cap) continue;
            uint32_t leftover = cap - piece_len;
            if (leftover < best_leftover) {
                best_leftover = leftover;
                best_i = i;
            }
        }

        if (best_i != SIZE_MAX) {
            plan_assign(p, in, best_i, piece_idx);
        } else {
            p->all_assigned = 0;
        }
    }

    free(L_sorted);
    plan_finalize(p, in);
    return p;
}

/* ---------------------------------------------------------------------- */
/*  Метод 3: локальный поиск (улучшение готового плана обменами)           */
/* ---------------------------------------------------------------------- */

/* Пробует переместить одну деталь piece_j с заготовки src на заготовку dst
 * (если она туда физически влезает без переупаковки остальных деталей src).
 * Возвращает 1, если перемещение выполнено. */
static int try_move_piece(plan_t *p, const input_t *in, size_t src, size_t pos_in_src, size_t dst)
{
    stock_cut_t *s_src = &p->stocks[src];
    size_t piece_idx = s_src->piece_idx[pos_in_src];
    uint32_t piece_len = in->L[piece_idx];

    /* Сколько освободится на src после удаления детали (учитывая, что
     * число резов на src уменьшится на 1) — нам не нужно вычислять явно,
     * достаточно проверить, влезает ли деталь на dst "как есть". */
    if (!fits(p, in, dst, piece_len)) return 0;

    /* удаляем деталь с src (сдвигаем хвост массива) */
    memmove(&s_src->piece_idx[pos_in_src], &s_src->piece_idx[pos_in_src + 1],
            (s_src->count - pos_in_src - 1) * sizeof(size_t));
    s_src->count--;
    s_src->consumed -= piece_len;

    plan_assign(p, in, dst, piece_idx);
    return 1;
}

/*
 * @description Локальный поиск first-improvement поверх готового плана:
 *              пытается переносить отдельные детали между заготовками,
 *              принимая перенос, только если он строго уменьшает
 *              суммарный отход (или, при равном отходе, число резов).
 *              Останавливается, когда за полный проход по всем деталям
 *              не найдено ни одного улучшения, либо по достижении
 *              max_passes проходов (защита от зацикливания).
 */
static void local_search_improve(const input_t *in, plan_t *p, int max_passes)
{
    if (!p->all_assigned) return; /* улучшать нечего — план и так неполный */

    for (int pass = 0; pass < max_passes; pass++) {
        int improved = 0;

        for (size_t src = 0; src < p->M; src++) {
            for (size_t pos = 0; pos < p->stocks[src].count; /* инкремент внутри */) {
                uint64_t waste_before = p->total_waste;
                uint64_t cuts_before  = p->total_cuts;
                int moved_here = 0;

                for (size_t dst = 0; dst < p->M; dst++) {
                    if (dst == src) continue;

                    /* пробуем перенос "на пробу": делаем, пересчитываем,
                     * откатываем, если не понравилось */
                    size_t piece_idx = p->stocks[src].piece_idx[pos];
                    if (!try_move_piece(p, in, src, pos, dst)) continue;

                    plan_finalize(p, in);
                    if (p->total_waste < waste_before ||
                        (p->total_waste == waste_before && p->total_cuts < cuts_before)) {
                        improved = 1;
                        moved_here = 1;
                        break; /* деталь перенесена, к следующей позиции src */
                    } else {
                        /* откат: переносим деталь обратно на src */
                        stock_cut_t *s_dst = &p->stocks[dst];
                        size_t last = s_dst->count - 1;
                        s_dst->count--;
                        s_dst->consumed -= in->L[piece_idx];
                        (void)last;
                        plan_assign(p, in, src, piece_idx);
                        /* деталь снова в src, но, возможно, не на прежней
                         * позиции — это не важно для корректности плана */
                        plan_finalize(p, in);
                    }
                }

                if (!moved_here) pos++; /* иначе перепроверяем ту же позицию */
            }
        }

        if (!improved) break;
    }

    plan_finalize(p, in);
    p->method_name = "BFD + локальный поиск";
}

/* ---------------------------------------------------------------------- */
/*  Метод 4: точный branch-and-bound для небольших M, N                    */
/* ---------------------------------------------------------------------- */

typedef struct {
    const input_t *in;
    indexed_u32_t *L_sorted;   /* детали по убыванию, с исходными индексами */
    plan_t *current;
    plan_t *best;
} bnb_ctx_t;

static void bnb_recurse(bnb_ctx_t *ctx, size_t depth)
{
    if (depth == ctx->in->N) {
        plan_finalize(ctx->current, ctx->in);
        ctx->current->all_assigned = 1;
        if (compare_plans(ctx->current, ctx->best) > 0) {
            free_plan(ctx->best);
            /* глубокая копия текущего плана в best */
            plan_t *copy = plan_alloc(ctx->in->M, "Точный перебор (branch-and-bound)");
            for (size_t i = 0; i < ctx->in->M; i++) {
                stock_cut_t *src = &ctx->current->stocks[i];
                for (size_t k = 0; k < src->count; k++)
                    plan_assign(copy, ctx->in, i, src->piece_idx[k]);
            }
            plan_finalize(copy, ctx->in);
            copy->all_assigned = 1;
            ctx->best = copy;
        }
        return;
    }

    uint32_t piece_len = ctx->L_sorted[depth].value;
    size_t   piece_idx = ctx->L_sorted[depth].orig_idx;

    /* нижняя граница отсечения: текущий отход уже не меньше best ->
     * дальнейшее добавление деталей отход только увеличит либо оставит
     * прежним, улучшить уже не сможем (при равенстве проверяем resend
     * по числу резов отдельно — здесь для простоты отсекаем только по
     * строгому превышению) */
    plan_finalize(ctx->current, ctx->in); /* оценка "как если бы уже финал" */
    if (ctx->best->all_assigned && ctx->current->total_waste > ctx->best->total_waste)
    {
        /* частичный отход - не окончательный (в оставшихся заготовках
         * еще appear могут появиться детали), поэтому это НЕ строгая
         * нижняя граница, а лишь эвристическое отсечение; для полной
         * корректности можно отключить проверку ниже, оставив только
         * полный перебор */
    }

    for (size_t i = 0; i < ctx->in->M; i++) {
        if (!fits(ctx->current, ctx->in, i, piece_len)) continue;

        /* симметрия: не пробуем повторно ту же по остаточной ёмкости и
         * тому же K заготовку — грубое, но полезное сокращение перебора
         * для наборов с повторяющимися длинами заготовок */
        int skip_symmetric = 0;
        for (size_t prev = 0; prev < i; prev++) {
            if (ctx->in->K[prev] == ctx->in->K[i] &&
                ctx->current->stocks[prev].count == ctx->current->stocks[i].count &&
                ctx->current->stocks[prev].consumed == ctx->current->stocks[i].consumed) {
                skip_symmetric = 1;
                break;
            }
        }
        if (skip_symmetric) continue;

        plan_assign(ctx->current, ctx->in, i, piece_idx);
        bnb_recurse(ctx, depth + 1);

        /* откат */
        stock_cut_t *sc = &ctx->current->stocks[i];
        sc->count--;
        sc->consumed -= piece_len;
    }
}

/*
 * @description Точный перебор с отсечением симметричных заготовок.
 *              ВНИМАНИЕ: экспоненциальная сложность (до M^N ветвлений в
 *              худшем случае). Используйте только при небольших M и N
 *              (см. EXACT_SIZE_LIMIT) — иначе вернёт NULL.
 * @returns Указатель на найденный оптимальный план, либо NULL, если
 *          размерность превышает EXACT_SIZE_LIMIT, либо решения не
 *          существует (тогда возвращается план с all_assigned == 0).
 */
static plan_t *solve_exact_bnb(const input_t *in)
{
    if (in->M + in->N > EXACT_SIZE_LIMIT) return NULL;

    bnb_ctx_t ctx;
    ctx.in = in;
    ctx.L_sorted = make_sorted_desc(in->L, in->N);
    if (!ctx.L_sorted) return NULL;

    ctx.current = plan_alloc(in->M, "(промежуточный)");
    ctx.best    = plan_alloc(in->M, "Точный перебор (branch-and-bound)");
    ctx.best->all_assigned = 0; /* пока решения не найдено */

    bnb_recurse(&ctx, 0);

    free_plan(ctx.current);
    free(ctx.L_sorted);
    return ctx.best; /* если решения не нашлось, all_assigned остался 0 */
}

/* ---------------------------------------------------------------------- */
/*  Вывод результата                                                       */
/* ---------------------------------------------------------------------- */

static void print_plan(const input_t *in, const plan_t *p)
{
    printf("\n=== %s ===\n", p->method_name);

    if (!p->all_assigned) {
        printf("Материала не хватает: этим методом не удалось разместить "
               "все %zu требуемых деталей на %zu заготовках (с учётом "
               "запаса на рез h=%u).\n", in->N, in->M, in->h);
        return;
    }

    int any_fully_unused = 0;

    for (size_t i = 0; i < p->M; i++) {
        const stock_cut_t *sc = &p->stocks[i];
        printf("K%-3zu=%-6u  ", i + 1, in->K[i]);
        if (sc->count == 0) {
            printf("(не использована, R%zu=%u)\n", i + 1, sc->remainder);
            any_fully_unused = 1;
            continue;
        }
        for (size_t k = 0; k < sc->count; k++) {
            printf("L%zu(%u) ", sc->piece_idx[k] + 1, in->L[sc->piece_idx[k]]);
        }
        printf(" (R%zu=%u)\n", i + 1, sc->remainder);
    }

    printf("Итого: суммарный отход = %llu мм, суммарное число резов = %llu\n",
           (unsigned long long)p->total_waste, (unsigned long long)p->total_cuts);

    if (any_fully_unused) {
        printf("Внимание: в этом плане есть заготовки, оставшиеся полностью "
               "неиспользованными — фактически можно было обойтись меньшим "
               "числом закупленных заготовок.\n");
    }
}

/* ---------------------------------------------------------------------- */
/*  main                                                                    */
/* ---------------------------------------------------------------------- */

int main(int argc, char **argv)
{
    input_t in;
    if (parse_args(argc, argv, &in) != 0) {
        fprintf(stderr,
            "\nИспользование: %s [-h H] M K1 K2 ... KM N L1 L2 ... LN\n"
            "  M       — число заготовок (натуральное)\n"
            "  K1..KM  — длины заготовок\n"
            "  N       — число требуемых деталей (натуральное)\n"
            "  L1..LN  — длины требуемых деталей\n"
            "  -h H    — необязательный технологический запас на рез (по умолчанию 0)\n"
            "\nПример:\n  %s -h 3 3 4701 8808 456 4 3205 740 740 740\n",
            argv[0], argv[0]);
        return EXIT_FAILURE;
    }

    if (check_feasibility(&in) != 0) {
        free_input(&in);
        return EXIT_FAILURE;
    }

    /* --- 1. Опорный план --- */
    plan_t *ffd = build_reference_plan_ffd(&in);
    print_plan(&in, ffd);

    /* --- 2. Эвристика получше --- */
    plan_t *bfd = build_plan_bfd(&in);
    print_plan(&in, bfd);

    /* --- 3. Локальный поиск поверх BFD --- */
    plan_t *improved = build_plan_bfd(&in); /* независимая копия для улучшения */
    local_search_improve(&in, improved, /*max_passes=*/20);
    print_plan(&in, improved);

    /* --- 4. Точный перебор (только для небольших размерностей) --- */
    plan_t *exact = solve_exact_bnb(&in);
    if (exact) {
        print_plan(&in, exact);
    } else {
        printf("\n=== Точный перебор (branch-and-bound) ===\n"
               "Пропущен: M+N=%zu превышает порог EXACT_SIZE_LIMIT=%d, "
               "перебор занял бы неприемлемо много времени.\n",
               in.M + in.N, EXACT_SIZE_LIMIT);
    }

    /* --- Итоговое сравнение и выбор лучшего из посчитанных планов --- */
    plan_t *candidates[4] = { ffd, bfd, improved, exact };
    const char *labels[4] = { "FFD", "BFD", "BFD+локальный поиск", "Точный перебор" };
    plan_t *best = NULL;
    const char *best_label = NULL;
    for (int i = 0; i < 4; i++) {
        if (!candidates[i]) continue;
        if (!best || compare_plans(candidates[i], best) > 0) {
            best = candidates[i];
            best_label = labels[i];
        }
    }

    printf("\n=== Лучший из рассчитанных планов: %s ===\n", best_label ? best_label : "нет допустимого плана");
    if (best && best->all_assigned) {
        printf("Суммарный отход = %llu мм, суммарное число резов = %llu\n",
               (unsigned long long)best->total_waste, (unsigned long long)best->total_cuts);
    } else {
        printf("Ни одним из методов не удалось построить допустимый план.\n");
    }

    free_plan(ffd);
    free_plan(bfd);
    free_plan(improved);
    free_plan(exact);
    free_input(&in);
    return EXIT_SUCCESS;
}
