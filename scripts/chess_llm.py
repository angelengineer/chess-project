import pygame
import chess
import chess.engine
import sys
import shutil
import ollama
import rclpy
from ros2_ws.src.chess_manager.chess_manager import node
from std_msgs.msg import String

# --- CONFIGURACIÓN ---
ANCHO = 600
ALTO = 600
DIMENSION = 8
TAM_CASILLA = ANCHO // DIMENSION
MAX_FPS = 15
TIEMPO_PENSAR_IA = 0.5  # Segundos que piensa la máquina

# Colores
BLANCO = (240, 217, 181)
NEGRO = (181, 136, 99)
RESALTE = (186, 202, 68)  # Verde lima para selección
ULTIMO_MOV = (255, 255, 100) # Amarillo suave


def pedir_feedback(board, ultimo_mov, rol):
    prompt = f"""
                Eres un jugador de ajedrez fuerte y hablas como un rival humano.
                Comenta brevemente (1-2 frases) la jugada reciente.

                Rol: {rol}
                Movimiento: {ultimo_mov}
                Posición (FEN): {board.fen()}
                """

    response = ollama.chat(
        model="llama3:8b",
        messages=[
            {"role": "system", "content": "Eres un rival de ajedrez que da feedback natural."},
            {"role": "user", "content": prompt}
        ]
    )

    return response["message"]["content"]

# --- LÓGICA DE STOCKFISH ---
def iniciar_motor():
    ruta = shutil.which("stockfish") or "/usr/games/stockfish"
    try:
        engine = chess.engine.SimpleEngine.popen_uci(ruta)
        print(f"✅ Motor cargado: {ruta}")
        return engine
    except FileNotFoundError:
        print("❌ ERROR: No se encontró Stockfish. Instálalo con 'sudo apt install stockfish'")
        sys.exit()

# --- INTERFAZ GRÁFICA ---
def dibujar_tablero(pantalla, seleccionada, ultimo_movimiento):
    """ Dibuja casillas y resaltados """
    for r in range(DIMENSION):
        for c in range(DIMENSION):
            color = BLANCO if (r + c) % 2 == 0 else NEGRO
            rect = pygame.Rect(c*TAM_CASILLA, r*TAM_CASILLA, TAM_CASILLA, TAM_CASILLA)
            
            # Dibujar casilla base
            pygame.draw.rect(pantalla, color, rect)

            # Resaltar último movimiento (si existe)
            if ultimo_movimiento and (
                (r == 7 - chess.square_rank(ultimo_movimiento.from_square) and c == chess.square_file(ultimo_movimiento.from_square)) or
                (r == 7 - chess.square_rank(ultimo_movimiento.to_square) and c == chess.square_file(ultimo_movimiento.to_square))
            ):
                pygame.draw.rect(pantalla, ULTIMO_MOV, rect)

            # Resaltar selección actual
            if seleccionada is not None:
                fila_sel, col_sel = seleccionada
                if r == fila_sel and c == col_sel:
                    pygame.draw.rect(pantalla, RESALTE, rect)

def dibujar_piezas(pantalla, board, fuente):
    """ Dibuja las piezas usando caracteres Unicode """
    simbolos = {
        'R': '♖', 'N': '♘', 'B': '♗', 'Q': '♕', 'K': '♔', 'P': '♙',
        'r': '♜', 'n': '♞', 'b': '♝', 'q': '♛', 'k': '♚', 'p': '♟'
    }
    
    for i in range(64):
        pieza = board.piece_at(i)
        if pieza:
            # Convertir índice de 0-63 a fila/col visual
            col = chess.square_file(i)
            row = 7 - chess.square_rank(i) # Pygame dibuja de arriba a abajo, ajedrez es al revés
            
            simbolo = simbolos[pieza.symbol()]
            
            # Color de la pieza (Negro puro o Blanco con borde negro simulado)
            if pieza.color == chess.BLACK:
                color_texto = (0, 0, 0)
            else:
                color_texto = (255, 255, 255) # Blanco brillante

            texto = fuente.render(simbolo, True, color_texto)
            
            # Si es blanca, dibujamos un borde negro pequeño para que se vea bien en casillas claras
            if pieza.color == chess.WHITE:
                texto_borde = fuente.render(simbolo, True, (0,0,0))
                rect_borde = texto_borde.get_rect(center=(col*TAM_CASILLA + TAM_CASILLA//2 + 2, row*TAM_CASILLA + TAM_CASILLA//2 + 2))
                pantalla.blit(texto_borde, rect_borde)

            rect_texto = texto.get_rect(center=(col*TAM_CASILLA + TAM_CASILLA//2, row*TAM_CASILLA + TAM_CASILLA//2))
            pantalla.blit(texto, rect_texto)

def main():
    pygame.init()
    rclpy.init()
    node = rclpy.create_node('chess_gui')
    pub = node.create_publisher(String, '/human_move', 10)

    pantalla = pygame.display.set_mode((ANCHO, ALTO))
    pygame.display.set_caption("Ajedrez vs IA (Python)")
    reloj = pygame.time.Clock()
    
    # Fuentes: Intentamos usar DejaVu Sans (común en Linux) o Arial
    try:
        fuente = pygame.font.SysFont("dejavusans", int(TAM_CASILLA * 0.9))
    except:
        fuente = pygame.font.SysFont("arial", int(TAM_CASILLA * 0.9))

    board = chess.Board()
    engine = iniciar_motor()
    
    casilla_seleccionada = None # Tupla (fila, col)
    casilla_origen_chess = None # Índice chess (0-63)
    ultimo_movimiento = None

    ejecutando = True
    game_over = False

    while ejecutando:
        turno_humano = (board.turn == chess.WHITE)
        
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                ejecutando = False
            
            # --- LOGICA DEL CLIC DEL HUMANO ---
            if turno_humano and not game_over and evento.type == pygame.MOUSEBUTTONDOWN:
                location = pygame.mouse.get_pos() # (x, y)
                col = location[0] // TAM_CASILLA
                row = location[1] // TAM_CASILLA
                
                # Si ya seleccionaste una pieza, este clic es el destino
                if casilla_seleccionada:
                    # Convertir coordenadas visuales a movimiento de ajedrez
                    casilla_destino_chess = chess.square(col, 7 - row)
                    
                    # Crear el movimiento
                    movimiento = chess.Move(casilla_origen_chess, casilla_destino_chess)
                    
                    # Manejo de PROMOCIÓN (Peón llega al final) -> Auto Reina
                    if board.piece_at(casilla_origen_chess) and board.piece_at(casilla_origen_chess).piece_type == chess.PAWN:
                        if (chess.square_rank(casilla_destino_chess) == 7): # Fila 8
                            movimiento = chess.Move(casilla_origen_chess, casilla_destino_chess, promotion=chess.QUEEN)

                    # Verificar si es legal
                    if movimiento in board.legal_moves:
                        msg = String()
                        msg.data = movimiento.uci()
                        pub.publish(msg)

                        board.push(movimiento)
                        ultimo_movimiento = movimiento
                        casilla_seleccionada = None
                        casilla_origen_chess = None
                        feedback = pedir_feedback(board, movimiento, "humano")
                        print("💬 Rival:", feedback)

                    else:
                        # Si el movimiento es ilegal, cambiamos la selección a la nueva casilla (si es propia)
                        casilla_seleccionada = (row, col)
                        casilla_origen_chess = chess.square(col, 7 - row)
                        
                else:
                    # Primer clic: Seleccionar pieza
                    casilla_seleccionada = (row, col)
                    casilla_origen_chess = chess.square(col, 7 - row)
                    # Si haces clic en una casilla vacía o enemiga, deseleccionar
                    pieza = board.piece_at(casilla_origen_chess)
                    if not pieza or pieza.color != chess.WHITE:
                        casilla_seleccionada = None

        # --- DIBUJAR ---
        dibujar_tablero(pantalla, casilla_seleccionada, ultimo_movimiento)
        dibujar_piezas(pantalla, board, fuente)
        
        if game_over:
             # Texto de fin de juego
            fuente_fin = pygame.font.SysFont("arial", 32, bold=True)
            texto = fuente_fin.render("JUEGO TERMINADO", True, (255, 0, 0))
            rect = texto.get_rect(center=(ANCHO//2, ALTO//2))
            pantalla.blit(texto, rect)

        pygame.display.flip()

        # --- TURNO DE LA IA ---
        if not turno_humano and not game_over:
            # Pequeña pausa para que se dibuje el movimiento del humano antes de que la IA piense
            pygame.time.wait(100) 
            
            result = engine.play(board, chess.engine.Limit(time=TIEMPO_PENSAR_IA))
            board.push(result.move)
            ultimo_movimiento = result.move
        
        # Verificar fin del juego
        if board.is_game_over():
            game_over = True

        reloj.tick(MAX_FPS)

    engine.quit()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()