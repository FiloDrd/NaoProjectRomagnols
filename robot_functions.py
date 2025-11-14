import time
import subprocess
import vlc

def do_moves(moves, robot_ip, port):
    """
    Esegue tutte le mosse nella lista e opzionalmente traccia i tempi di esecuzione.
    
    Args:
        moves: lista delle mosse da eseguire
        robot_ip: IP del robot
        port: porta del robot
        track_timing: se True, salva i tempi in moves_length.txt
    """    
    start_choreography_time = time.time()
    for move in moves:
        print(f"Executing: {move}... ", end="", flush=True)
        
        # Misura il tempo di inizio
        start_move_time = time.time()
        
        # Esegui la mossa
        python2_command = f"python2 ./moves/{move}.py  {robot_ip} {port}"
        process = subprocess.run(python2_command.split(), stdout=subprocess.PIPE)
        
        # Misura il tempo di fine
        end_move_time = time.time()
        elapsed_move_time = end_move_time - start_move_time
        print(f"OK ({elapsed_move_time:.2f}s)")
    end_choreography_time = time.time()
    total_elapsed_time = end_choreography_time - start_choreography_time
    print(f"Choreography completed in {total_elapsed_time:.2f}s")
    
def play_song(song_name):
    p = vlc.MediaPlayer(song_name)
    p.play()
    pass