import time
import subprocess

def do_moves(moves, robot_ip, port, track_timing=True):
    """
    Esegue tutte le mosse nella lista e opzionalmente traccia i tempi di esecuzione.
    
    Args:
        moves: lista delle mosse da eseguire
        robot_ip: IP del robot
        port: porta del robot
        track_timing: se True, salva i tempi in moves_length.txt
    """    
    for move in moves:
        print(f"Executing: {move}... ", end="", flush=True)
        
        # Misura il tempo di inizio
        start_time = time.time()
        
        # Esegui la mossa
        python2_command = f"python2 ./moves/{move}.py  {robot_ip} {port}"
        process = subprocess.run(python2_command.split(), stdout=subprocess.PIPE)
        
        # Misura il tempo di fine
        end_time = time.time()
        elapsed_time = end_time - start_time
        print(f"OK ({elapsed_time:.2f}s)")
    