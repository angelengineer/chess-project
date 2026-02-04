import chess
import chess.engine

def jugar_ajedrez():
    # 1. Configurar el tablero
    board = chess.Board()
    
    # 2. Conectar con Stockfish
    # IMPORTANTE: Verifica que esta ruta sea correcta con el comando 'which stockfish' en terminal
    ruta_stockfish = "/usr/games/stockfish" 
    
    try:
        engine = chess.engine.SimpleEngine.popen_uci(ruta_stockfish)
    except FileNotFoundError:
        print(f"Error: No encontré Stockfish en {ruta_stockfish}")
        print("Ejecuta 'which stockfish' en tu terminal para ver la ruta correcta.")
        return

    # Nivel de dificultad (0.1 segundos por jugada es rápido pero fuerte para humanos)
    limite = chess.engine.Limit(time=0.5)

    print("¡Bienvenido! Juegas con Blancas.")
    print("Introduce movimientos en notación algebraica (ej: e4, Nf3) o coordenadas (ej: e2e4).")
    print("--------------------------------------------------------------------")

    while not board.is_game_over():
        # Mostrar tablero
        print("\n" + str(board))
        print("\nTu turno:")

        # --- Turno del Humano ---
        while True:
            try:
                mov_texto = input("Introduce tu movimiento: ")
                # Intentar interpretar el movimiento (ej: "e4" o "e2e4")
                board.push_san(mov_texto)
                break
            except ValueError:
                print("Movimiento inválido o mal escrito. Intenta de nuevo.")

        if board.is_game_over():
            break

        # --- Turno de la Máquina ---
        print("\nLa máquina está pensando...")
        result = engine.play(board, limite)
        board.push(result.move)
        print(f"La máquina jugó: {result.move}")

    # Fin del juego
    print("\n¡Juego terminado!")
    print("Resultado:", board.result())
    engine.quit()

if __name__ == "__main__":
    jugar_ajedrez()