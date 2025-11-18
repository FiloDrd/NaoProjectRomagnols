#!/usr/bin/env python3
"""
Choreography Planner per NAO Robot
Implementa un algoritmo best-first/A* per pianificare sequenze di mosse
che rispettino i vincoli di precedenza e si avvicinino al tempo target.
"""

import heapq
import json
import sys
from typing import List, Dict, Tuple, Optional, Set
import robot_functions as rf


# ===== CONFIGURAZIONE INPUT (MODIFICABILE) =====

# Goals obbligatori da eseguire in questo ordine
goals = [
    "StandInit",      
    "Hello",        
    "BlowKisses",     
    "StayingAlive",  
    "WipeForehead",
    "Sit",
    "StandZero",     
    "VOnEyes",    
    "SitRelax",
    "Stand",        
    "Crouch"
]

# Predecessori validi per ogni mossa (START indica inizio coreografia)
predecessors = {
    "StandInit": ["START"],
    
    "Hello": ["Wave", "Clap", "Glory", "Joy", "ComeOn", "Bow", "StandInit"],
    
    "BlowKisses": ["Stand", "ArmDance", "ArmDanceDX", "ArmDanceSX", "Bow", "Wave", "Hello"],
    
    "StandZero": ["Sit", "SitRelax", "RotationFeet", "ArmDanceDX", "ArmDanceSX", "BirthdayDance", 
                  "Hello", "Stand", "BlowKisses", "Wave", "Bow", "Glory"],
    
    "Rhythm": ["ArmDance", "ArmDanceDX", "ArmDanceSX", "Stand", "Bow", 
               "RotationFeet", "StandZero", "Wave"],
    
    "Stand": ["Sit", "SitRelax", "RotationFeet", "BirthdayDance", "Hello", "StandZero", "Rhythm",
              "WipeForehead", "Bow", "Wave", "Glory", "Clap", "Dab"],
    
    "StayingAlive": ["PulpFiction", "DiagonalRight", "RotationFeet", "TheRobot", 
                     "Bow", "Stand", "StandZero"],
    
    "WipeForehead": ["Stand", "StandZero", "Wave", "Glory", "Clap", "Bow", "StayingAlive"],
    
    "VOnEyes": ["BlowKisses", "Hello", "Wave", "Joy", "ComeOn", "Stand", "WipeForehead"],

    "Sit": ["Dab", "Stand", "StandZero", "VOnEyes"],

    "SitRelax": ["PulpFiction", "DiagonalRight", "RotationFeet", "TheRobot", 
                     "Bow", "Stand", "StandZero"],

    "Disco": ["Stand", "StandZero", "TheRobot", "PulpFiction", 
              "DiagonalRight", "RotationFeet", "Crouch"],

    "Crouch": ["SitRelax", "Crouch", "Dab", "Disco"],

    "Wave": ["START", "StandInit", "Hello", "Stand", "StandZero", "Clap", 
             "Glory", "Joy", "ComeOn", "Bow"],
    "Clap": ["START", "StandInit", "Wave", "Hello", "Stand", "Joy", "ComeOn", 
             "Glory", "Bow", "StandZero"],
    "Glory": ["Wave", "Clap", "Stand", "StandZero", "Bow", "Joy", "ComeOn"],
    "Joy": ["Clap", "Glory", "Hello", "Wave", "ComeOn", "Bow", "Stand"],
    "ComeOn": ["Clap", "Joy", "Hello", "Wave", "Glory", "Bow", "Stand"],
    "Bow": ["Stand", "StandZero", "Glory", "Wave", "Clap", "Joy", "ComeOn"],
    
    # Arm movements
    "ArmDance": ["Stand", "StandZero", "Bow", "Wave", "Clap", "Glory"],
    "ArmDanceDX": ["Stand", "Bow", "Wave", "Glory", "Clap", "StandZero"],
    "ArmDanceSX": ["Stand", "Bow", "Wave", "Glory", "Clap", "StandZero"],
    
    # Rhythmic/Dance movements
    "PulpFiction": ["Stand", "StandZero", "DiagonalRight", "RotationFeet", 
                    "Wave", "Clap", "Bow"],
    "RotationFeet": ["START", "StandInit", "Stand", "StandZero", "DiagonalRight", 
                     "Wave", "Glory", "Bow"],
    "DiagonalRight": ["Stand", "StandZero", "RotationFeet", "PulpFiction", 
                      "Wave", "Bow", "Clap"],
    "TheRobot": ["Stand", "StandZero", "PulpFiction", "DiagonalRight", 
                 "RotationFeet", "Bow"],
    "BirthdayDance": ["Stand", "StandZero", "Wave", "Clap", "Bow"],
    
    # Ponti critici per transizioni
    "Dab": ["Stand", "StandZero", "Bow", "Glory", "Wave", "Clap", 
              "WipeForehead", "VOnEyes"],
    
    "DanceMove": ["Stand", "StandZero", "Wave", "Bow", "Clap"],
}

# Durate in secondi per ciascuna mossa
durations = {
    "ArmDance": 10.353,
    "ArmDanceDX": 5.097,
    "BlowKisses": 9.867,
    "Bow": 3.994,
    "Clap": 4.180,
    "Crouch": 5.435,
    "Dab": 3.231,
    "Disco": 5.152,
    "Rhythm": 3.099,
    "StandInit": 1.410,
    "StayingAlive": 6.030,
    "VOnEyes": 6.049
}

# ===== PARAMETRI DI CONTROLLO =====
TARGET_TIME = 110.0          # Tempo target in secondi
MIN_TIME = 100.0             # Tempo minimo accettabile
MAX_TIME = 125.0             # Tempo massimo accettabile
DEFAULT_DURATION = 5.0       # Durata default per mosse non definite
MAX_EXPANSIONS = 10000       # Limite espansioni per evitare esplosione
MAX_STEPS = 50               # Lunghezza massima sequenza
MAX_CONSECUTIVE_REPEATS = 3  # Max ripetizioni consecutive stessa mossa
ALLOW_POST_FILL = True       # Permetti filler dopo completamento goals
KEEP_TOP_K = 5               # Numero soluzioni alternative da mostrare


# ===== STRUTTURE DATI =====

class State:
    """Rappresenta uno stato nella ricerca della coreografia."""
    
    def __init__(self, sequence: List[str], goals_completed: int, total_time: float, filler_moves: int):
        self.sequence = sequence
        self.goals_completed = goals_completed
        self.total_time = total_time
        self.filler_moves = filler_moves # <-- NUOVA RIGA
        # Euristica: tempo rimanente stimato per i goal
        self.heuristic_value = self._compute_heuristic()
        
        # === LOGICA DI PRIORITÀ A 3 LIVELLI ===

        # 1. Penalità GOAL (Priorità Massima: 10000.0)
        # Deve essere enorme per garantire che il completamento dei goal
        # sia SEMPRE più importante di tutto il resto.
        goal_penalty = (len(goals) - self.goals_completed) * 10000.0

        # 2. Penalità FILLER (Priorità Media: 100.0)
        # Si applica solo se i goal sono finiti ma mancano i filler.
        # Guida la ricerca (dopo i goal) ad aggiungere filler.
        filler_penalty = 0.0
        if self.goals_completed == len(goals) and self.filler_moves < 5:
            filler_penalty = (5 - self.filler_moves) * 100.0

        # 3. Penalità TEMPO (Priorità Bassa: 0-50+)
        # Guida la ricerca verso il range 110-125s.
        # Usiamo il tempo *stimato* (attuale + euristica)
        time_penalty = 0.0
        estimated_final_time = self.total_time + self.heuristic_value
        
        if estimated_final_time < MIN_TIME:
            # Penalizza quanto siamo lontani dal minimo
            time_penalty = (MIN_TIME - estimated_final_time)
        elif estimated_final_time > MAX_TIME:
            # Penalizza quanto siamo lontani dal massimo
            time_penalty = (estimated_final_time - MAX_TIME)
        # Se è nel range 110-125, la penalità è 0 (perfetto!)

        # La priorità totale è la somma delle penalità.
        # L'algoritmo cercherà di minimizzare questo valore.
        self.priority = goal_penalty + filler_penalty + time_penalty

    def _compute_heuristic(self) -> float:
        """Euristica ammissibile: somma durate goals rimanenti."""
        remaining_goals = goals[self.goals_completed:]
        return sum(durations.get(goal, DEFAULT_DURATION) for goal in remaining_goals)
    
    def __lt__(self, other):
        # Min-heap, quindi priorità più bassa è migliore
        return self.priority < other.priority

# ===== FUNZIONI HELPER =====

def get_move_duration(move: str) -> float:
    """Ottiene la durata di una mossa."""
    return durations.get(move, DEFAULT_DURATION)

def can_follow(prev_move: str, next_move: str) -> bool:
    """Verifica se next_move può seguire prev_move."""
    if next_move not in predecessors:
        return False
    return prev_move in predecessors[next_move]

def get_valid_next_moves(current_move: str, goals_completed: int, sequence: List[str]) -> List[str]:
    """Ottiene le mosse valide che possono seguire current_move."""
    valid_moves = []
    
    # Se abbiamo completato tutti i goals e non permettiamo filler, stop
    if goals_completed >= len(goals) and not ALLOW_POST_FILL:
        return []
    
    # Se il prossimo goal può seguire direttamente, aggiungerlo con priorità
    if goals_completed < len(goals):
        next_goal = goals[goals_completed]
        if can_follow(current_move, next_goal):
            valid_moves.append(next_goal)
    
    # Aggiungi mosse intermedie permesse (non goals già completati)
    completed_goals_set = set(goals[:goals_completed])
    
    for move, move_preds in sorted(predecessors.items()):
        if (move not in completed_goals_set and 
            current_move in move_preds):  # Evita self-loop
            
            # Controllo ripetizioni consecutive
            if len(sequence) >= MAX_CONSECUTIVE_REPEATS:
                recent_moves = sequence[-MAX_CONSECUTIVE_REPEATS:]
                if all(m == move for m in recent_moves):
                    continue
            
            valid_moves.append(move)
    
    return list(set(valid_moves))  # Rimuovi duplicati

def expand_state(state: State) -> List[State]:
    """Espande uno stato generando tutti i possibili stati successivi."""
    if len(state.sequence) >= MAX_STEPS:
        return []
    
    current_move = state.sequence[-1] if state.sequence else "START"
    next_moves = get_valid_next_moves(current_move, state.goals_completed, state.sequence)
    
    expanded_states = []
    
    for move in next_moves:
        new_sequence = state.sequence + [move]
        new_time = state.total_time + get_move_duration(move)
        
        # Calcola goals completati E filler
        new_goals_completed = state.goals_completed
        new_filler_moves = state.filler_moves
        
        is_next_goal = (new_goals_completed < len(goals) and 
                        move == goals[new_goals_completed])
        
        if is_next_goal:
            new_goals_completed += 1
        else:
            # Se NON è il prossimo goal, è un filler
            # (anche se è un goal futuro o uno vecchio)
            new_filler_moves += 1
        
        # Passa il nuovo conteggio filler
        new_state = State(new_sequence, new_goals_completed, new_time, new_filler_moves)
        expanded_states.append(new_state)
    
    return expanded_states

def is_valid_solution(state: State) -> bool:
    """Verifica se uno stato rappresenta una soluzione valida."""
    return state.goals_completed >= len(goals)

# ===== ALGORITMO PRINCIPALE =====

def find_choreography() -> Tuple[Optional[State], List[State]]:
    """
    Trova la migliore coreografia usando algoritmo best-first/A*.
    Returns: (best_solution, alternative_solutions)
    """
    # Priority queue: (priority, counter, state)
    # Counter per evitare confronti tra stati con stessa priorità
    pq = []
    counter = 0
    visited = set()
    expansions = 0
    all_solutions = []
    
    # Stato iniziale
    initial_state = State([], 0, 0.0, 0) # Aggiunto 0 per filler_moves
    heapq.heappush(pq, (initial_state.priority, counter, initial_state))
    counter += 1
    
    best_solution = None
    
    while pq and expansions < MAX_EXPANSIONS:
        _, _, current_state = heapq.heappop(pq)
        
        # Evita stati già visitati (basato su sequenza e goals completati)
        state_key = (tuple(current_state.sequence), current_state.goals_completed)
        if state_key in visited:
            continue
        visited.add(state_key)
        
        # Verifica se è una soluzione
        if is_valid_solution(current_state):
            all_solutions.append(current_state)
            if best_solution is None or current_state.priority < best_solution.priority:
                best_solution = current_state
            continue
        
        # Espandi stato corrente
        for next_state in expand_state(current_state):
            heapq.heappush(pq, (next_state.priority, counter, next_state))
            counter += 1
        
        expansions += 1
    
    # Ordina soluzioni per qualità
    all_solutions.sort(key=lambda s: s.priority)
    alternatives = all_solutions[:KEEP_TOP_K] if len(all_solutions) > 1 else []
    
    return best_solution, alternatives

# ===== OUTPUT E MAIN =====

def save_solution(solution: State, filename: str = "choreography_output.json"):
    """Salva la soluzione in formato JSON."""
    output = {
        "total_time": round(solution.total_time, 2),
        "sequence": solution.sequence,
        "goals_completed": solution.goals_completed,
        "target_time": TARGET_TIME,
        "time_difference": round(abs(solution.total_time - TARGET_TIME), 2)
    }
    
    with open(filename, 'w', encoding='utf-8') as f:
        json.dump(output, f, indent=2, ensure_ascii=False)

def check_order(solution: State, goals: list):
    target_len = len(goals)
    c = 0
    for e in solution.sequence:
        if e == goals[c]:
            c += 1
    return c == target_len

def print_solution(solution: State, title: str = "Soluzione Migliore"):
    """Stampa una soluzione in modo leggibile."""
    print(f"\n=== {title} ===")
    print(f"Sequenza: {' → '.join(solution.sequence)}")
    print(f"Durata totale: {solution.total_time:.2f}s")
    print(f"Target: {TARGET_TIME}s")
    print(f"Differenza: {abs(solution.total_time - TARGET_TIME):.2f}s")
    print(f"Goals completati: {solution.goals_completed}/{len(goals)}")
    print(f"Mosse 'filler': {solution.filler_moves}") # <-- NUOVA RIGA
    # Verifica ordine goals
    goal_positions = []
    for i, goal in enumerate(goals):
        try:
            pos = solution.sequence.index(goal)
            goal_positions.append(pos)
        except ValueError:
            print(f"⚠️  Goal mancante: {goal}")
    if check_order(solution, goals):
        print("✅ Ordine goals rispettato")
    else:
        print(f"❌ Ordine goals NON rispettato:")
        if len(goal_positions) != len(goals):
            print(f"lunghezza goals trovati: {len(goal_positions)} vs trovati: {len(goals)}")
    
    if solution.filler_moves >= 5:
        print("✅ Numero filler rispettato (>= 5)")
    else:
        print(f"❌ Numero filler NON rispettato ({solution.filler_moves} < 5)")
    
    if 110.0 <= solution.total_time <= 125.0:
        print("✅ Tempo nel range ideale")
    else:
        print("❌ Tempo fuori dal range ideale")

def main():
    """Funzione principale."""
    print("=== CHOREOGRAPHY PLANNER per NAO Robot ===")
    print(f"Goals da completare: {len(goals)}")
    print(f"Target time: {TARGET_TIME}s")
    print(f"Max espansioni: {MAX_EXPANSIONS}")
    
    # Validazione configurazione
    missing_goals = [g for g in goals if g not in predecessors]
    if missing_goals:
        print(f"⚠️  Goals senza predecessori definiti: {missing_goals}")
        return
    
    print("\nCerca in corso...")
    best_solution, alternatives = find_choreography()
    
    if best_solution is None:
        print("❌ Nessuna soluzione trovata!")
        print("Suggerimenti:")
        print("- Verifica che tutti i goals abbiano predecessori validi")
        print("- Aumenta MAX_EXPANSIONS o MAX_STEPS")
        print("- Controlla la connettività del grafo di mosse")
        return
    
    # Output risultati
    print_solution(best_solution)
    
    # Salva su file
    save_solution(best_solution)
    print(f"\n💾 Soluzione salvata in 'choreography_output.json'")
    
    # Mostra alternative
    if alternatives:
        print(f"\n=== TOP {min(len(alternatives), KEEP_TOP_K)} ALTERNATIVE ===")
        for i, alt in enumerate(alternatives[:KEEP_TOP_K], 1):
            print_solution(alt, f"Alternativa {i}")
    
    print(f"\n🎯 Ricerca completata!")
    #TODO, passare la migliore coreografia al modulo di esecuzione del NAO Robot
    print("🚀 Esecuzione della coreografia sul NAO Robot in corso...")
    ip = sys.argv[1]
    port = sys.argv[2]
    rf.play_song("Drake - One Dance.mp3")
    rf.do_moves(best_solution.sequence, ip, port)
    
if __name__ == "__main__":
    main()
