import pygame
import chess
import chess.engine
import sys
import shutil

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# --- CONFIGURACIÓN ---
ANCHO = 600
ALTO = 600
DIMENSION = 8
TAM_CASILLA = ANCHO // DIMENSION
MAX_FPS = 15
TIEMPO_PENSAR_IA = 0.5  # segundos

# Colores
BLANCO = (240, 217, 181)
NEGRO = (181, 136, 99)
RESALTE = (186, 202, 68)
ULTIMO_MOV = (255, 255, 100)


# --- ROS NODE ---
class ChessGameNode(Node):
    def __init__(self):
        super().__init__('chess_game_node')
        self.move_pub = self.create_publisher(String, '/robot_move', 10)

    def publish_move(self, move: chess.Move, Color):
        msg = String()
        msg.data = f"{'white' if Color == chess.WHITE else 'black'}:{move.uci()}"
        self.move_pub.publish(msg)
        self.get_logger().info(f'Published move: {msg.data}')


# --- STOCKFISH ---
def iniciar_motor():
    ruta = shutil.which("stockfish") or "/usr/games/stockfish"
    try:
        engine = chess.engine.SimpleEngine.popen_uci(ruta)
        print(f"✅ Stockfish cargado: {ruta}")
        return engine
    except FileNotFoundError:
        print("❌ Stockfish no encontrado")
        sys.exit()


# --- DIBUJO ---
def dibujar_tablero(pantalla, seleccionada, ultimo_movimiento):
    for r in range(DIMENSION):
        for c in range(DIMENSION):
            color = BLANCO if (r + c) % 2 == 0 else NEGRO
            rect = pygame.Rect(c*TAM_CASILLA, r*TAM_CASILLA, TAM_CASILLA, TAM_CASILLA)
            pygame.draw.rect(pantalla, color, rect)

            if ultimo_movimiento and (
                (r == 7 - chess.square_rank(ultimo_movimiento.from_square)
                 and c == chess.square_file(ultimo_movimiento.from_square)) or
                (r == 7 - chess.square_rank(ultimo_movimiento.to_square)
                 and c == chess.square_file(ultimo_movimiento.to_square))
            ):
                pygame.draw.rect(pantalla, ULTIMO_MOV, rect)

            if seleccionada and (r, c) == seleccionada:
                pygame.draw.rect(pantalla, RESALTE, rect)


def dibujar_piezas(pantalla, board, fuente):
    simbolos = {
        'R': '♖', 'N': '♘', 'B': '♗', 'Q': '♕', 'K': '♔', 'P': '♙',
        'r': '♜', 'n': '♞', 'b': '♝', 'q': '♛', 'k': '♚', 'p': '♟'
    }

    for i in range(64):
        pieza = board.piece_at(i)
        if pieza:
            col = chess.square_file(i)
            row = 7 - chess.square_rank(i)
            simbolo = simbolos[pieza.symbol()]
            color = (0, 0, 0) if pieza.color == chess.BLACK else (255, 255, 255)
            texto = fuente.render(simbolo, True, color)
            rect = texto.get_rect(center=(col*TAM_CASILLA + TAM_CASILLA//2,
                                          row*TAM_CASILLA + TAM_CASILLA//2))
            pantalla.blit(texto, rect)


# --- MAIN ---
def main():
    rclpy.init()
    node = ChessGameNode()

    pygame.init()
    pantalla = pygame.display.set_mode((ANCHO, ALTO))
    pygame.display.set_caption("Ajedrez vs IA + ROS 2")
    reloj = pygame.time.Clock()

    fuente = pygame.font.SysFont("dejavusans", int(TAM_CASILLA * 0.9))

    board = chess.Board()
    engine = iniciar_motor()

    casilla_sel = None
    casilla_origen = None
    ultimo_movimiento = None
    ejecutando = True
    game_over = False

    while ejecutando:
        turno_humano = board.turn == chess.WHITE

        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                ejecutando = False

            if turno_humano and not game_over and evento.type == pygame.MOUSEBUTTONDOWN:
                x, y = pygame.mouse.get_pos()
                col = x // TAM_CASILLA
                row = y // TAM_CASILLA

                if casilla_sel:
                    destino = chess.square(col, 7 - row)
                    movimiento = chess.Move(casilla_origen, destino)

                    if board.piece_at(casilla_origen) and \
                       board.piece_at(casilla_origen).piece_type == chess.PAWN and \
                       chess.square_rank(destino) == 7:
                        movimiento = chess.Move(casilla_origen, destino,
                                                 promotion=chess.QUEEN)

                    if movimiento in board.legal_moves:
                        board.push(movimiento)
                        ultimo_movimiento = movimiento
                        # After push, board.turn is the next player; mover is the opposite.
                        node.publish_move(movimiento, not board.turn)

                        casilla_sel = None
                        casilla_origen = None
                    else:
                        casilla_sel = None
                        casilla_origen = None
                else:
                    origen = chess.square(col, 7 - row)
                    pieza = board.piece_at(origen)
                    if pieza and pieza.color == chess.WHITE:
                        casilla_sel = (row, col)
                        casilla_origen = origen

        dibujar_tablero(pantalla, casilla_sel, ultimo_movimiento)
        dibujar_piezas(pantalla, board, fuente)
        pygame.display.flip()

        if not turno_humano and not game_over:
            pygame.time.wait(100)
            result = engine.play(board, chess.engine.Limit(time=TIEMPO_PENSAR_IA))
            board.push(result.move)
            ultimo_movimiento = result.move
            # After push, board.turn is the next player; mover is the opposite.
            node.publish_move(result.move, not board.turn)

        if board.is_game_over():
            game_over = True

        rclpy.spin_once(node, timeout_sec=0.0)
        reloj.tick(MAX_FPS)

    engine.quit()
    pygame.quit()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit()


if __name__ == "__main__":
    main()
