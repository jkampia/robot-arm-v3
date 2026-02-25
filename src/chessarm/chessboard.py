from enum import Enum, IntEnum
import numpy as np
import copy

from chessboard import display

from stockfish import Stockfish


class Occupancy(IntEnum):
    EMPTY = 0
    WHITE = 1
    BLACK = 2


class PieceType(IntEnum):
    PAWN = 0
    ROOK = 1
    KNIGHT = 2
    BISHOP = 3
    QUEEN = 4
    KING = 5
    UNKNOWN = 6


class CastlingRights(IntEnum):
    BOTH = 3
    KINGSIDE = 2
    QUEENSIDE = 1
    NONE = 0


class MoveType(Enum):
    NORMAL = 0,
    EN_PASSANT = 1,
    CASTLE = 2,



StartingPieceOrder = {
    1: [PieceType.ROOK, PieceType.KNIGHT, PieceType.BISHOP, PieceType.QUEEN, PieceType.KING, PieceType.BISHOP, PieceType.KNIGHT, PieceType.ROOK],
    2: [PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN],
    3: [None, None, None, None, None, None, None, None],
    4: [None, None, None, None, None, None, None, None],
    5: [None, None, None, None, None, None, None, None],
    6: [None, None, None, None, None, None, None, None],
    7: [PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN],
    8: [PieceType.ROOK, PieceType.KNIGHT, PieceType.BISHOP, PieceType.QUEEN, PieceType.KING, PieceType.BISHOP, PieceType.KNIGHT, PieceType.ROOK]
}



StartingRowOccupancy = {
    1: Occupancy.BLACK,
    2: Occupancy.BLACK,
    3: Occupancy.EMPTY,
    4: Occupancy.EMPTY,
    5: Occupancy.EMPTY,
    6: Occupancy.EMPTY,
    7: Occupancy.WHITE,
    8: Occupancy.WHITE,
}



FENSymbols = {
    PieceType.PAWN: "p",
    PieceType.ROOK: "r",
    PieceType.KNIGHT: "n",
    PieceType.BISHOP: "b",
    PieceType.QUEEN: "q",
    PieceType.KING: "k"
}


FENCastling = {
    CastlingRights.BOTH: "KQ",
    CastlingRights.KINGSIDE: "K",
    CastlingRights.QUEENSIDE: "Q", 
    CastlingRights.NONE: ""
}





class ChessSquare:

    def __init__(self):

        self.occupancy = Occupancy.EMPTY
        self.piece_type = None


class ChessBoard:


    def __init__(self, playing_as):
        
        self.current_state = [[ChessSquare() for col in range(8)] for row in range(8)]
        self.prev_state = []
        self.setupInitialBoard(playing_as)
        self.snapshot() 
        self.stockfish = Stockfish(path="/usr/games/stockfish", depth=18, parameters={"Threads": 2, "Minimum Thinking Time": 30})

        self.playing_as = playing_as
        self.white_can_castle = CastlingRights.BOTH
        self.black_can_castle = CastlingRights.BOTH
        self.en_passant_targets = [] #contains list of coordinates (i, j) 
        self.halfmove_clock = 0
        self.fullmoves = 0

        self.injectTestCase()

        

    def setupInitialBoard(self, robot_color):
        for i in range(8):
            for j in range(8):
                self.current_state[i][j].occupancy = StartingRowOccupancy[i+1]
                self.current_state[i][j].piece_type = StartingPieceOrder[i+1][j]



    def snapshot(self):
        # make a full, independent copy of the board state
        self.prev_state = copy.deepcopy(self.current_state)


    
    def rotate_board_180(self):
        """
        which: 'current' or 'prev' — which board to rotate.
        also_prev: if True, rotate prev_state too (keeps diff orientation consistent).
        """
        def rot(s):
            return [[copy.deepcopy(s[7 - i][7 - j]) for j in range(8)] for i in range(8)]
        self.current_state = rot(self.current_state)
    


    def matrixSquareToStandardNotation(self, tuple):
        """
        Converts square in matrix notation (i, j) to standard 
        chess grid notation (e4)
        """
        horiz_list = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
        vert_list = ['8', '7', '6', '5', '4', '3', '2', '1']
        return f"{horiz_list[tuple[0]]}{vert_list[tuple[1]]}"
    


    def standardNotationToMatrixSquare(self, move):
        """
        Converts position in standard notation (e4) to 
        matrix notation (i, j)
        """
        if len(move) != 2:
            print("something wrong happened")
        horiz_dict = {'a': 0, 'b': 1, 'c': 2, 'd': 3, 'e': 4, 'f': 5, 'g': 6, 'h': 7}
        vert_dict = {'8': 0, '7': 1, '6': 2, '5': 3, '4': 4, '3': 5, '2': 6, '1': 7}
        return (vert_dict[move[1]], horiz_dict[move[0]])



    def stockfishMoveToMatrixMove(self, move):
        """
        Accepts standard move input of length 4 (i.e. e4d4)
        Converts it to matrix index notation, returns as list
        [(i1, j1), (i2, j2)]
        """
        tuple1 = self.standardNotationToMatrixSquare(move[0:2])
        tuple2 = self.standardNotationToMatrixSquare(move[2:4])
        return [tuple1, tuple2]


    
    def simulateMoves(self, moves):
        """
        Move piece from pos1 to pos2 where pos1 and pos2 are tuples (row, col)
        Must pass moves in as a list, even if it is a single move
        """
        self.snapshot() #save current board state before making changes

        for move in moves:

            if isinstance(move, str):
                move = self.stockfishMoveToMatrixMove(move)
                print(move)

            i1, j1 = move[0] #unpack
            i2, j2 = move[1]
        
            moved_square_occupancy = self.current_state[i1][j1].occupancy
            
            self.current_state[i2][j2].piece_type = PieceType.UNKNOWN #dont update this as robot wouldnt know this in real life
            self.current_state[i2][j2].occupancy = moved_square_occupancy

            self.current_state[i1][j1].piece_type = None #clear out old square
            self.current_state[i1][j1].occupancy = Occupancy.EMPTY

            self.fullmoves += 1 # inc move counter for FEN notation



    def toFEN(self, state):
        """
        Return current board state in FEN notation
        https://www.chess.com/terms/fen-chess
        """
        fenstring = ""
        for i in range(8):
            empty_counter = 0 #reset empty counter on each row
            for j in range(8):
                if state[i][j].piece_type is not None: #if square isn't empty
                    if empty_counter > 0: # if we have reached a piece, dump empty counter
                        fenstring += f"{empty_counter}"
                        empty_counter = 0
                    if state[i][j].occupancy == Occupancy.BLACK:
                        fenstring += FENSymbols[state[i][j].piece_type]
                    else:
                        fenstring += FENSymbols[state[i][j].piece_type].upper() #make uppercase if piece is white
                else:
                    empty_counter += 1
                    if j == 7: # if we are at the last col of the row, dump empty counter
                        fenstring += f"{empty_counter}"
                        empty_counter = 0

                if j == 7 and i < 7:
                    fenstring += "/"
        
        if self.playing_as == "white":
            fenstring += " w"
        else:
            fenstring += " b"

        fenstring += f" {FENCastling[self.white_can_castle]}"
        fenstring += FENCastling[self.black_can_castle].lower()
        fenstring += " " # add space before en passant targets

        if len(self.en_passant_targets) == 0:
            fenstring += "-"
        else:
            for tgt in self.en_passant_targets:
                fenstring += self.matrixSquareToStandardNotation(tgt)

        fenstring += f" {self.halfmove_clock}"

        fenstring += f" {self.fullmoves}"

        return fenstring
    


    def countPieces(self, state):
        """
        Count pieces on board state
        """
        piece_count = 0
        for i in range(8):
            for j in range(8):
                if state[i][j].piece_type is not None: piece_count += 1
        return piece_count
    


    def findWhichIndicesAreEmpty(self, idx_list):
        if len(idx_list) == 0:
            print(f'findWhichIndicesAreEmpty() was passed a list of length 0. This is wrong')
            return
            
        """
        each idx is a tuple
        expecting length 2, only 1 can be empty so return it
        """
        empty_indices = []
        occupied_indices = []
        for idx in idx_list:
            if self.current_state[idx[0]][idx[1]].piece_type is None:
                empty_indices.append(idx)
            else:
                occupied_indices.append(idx)

        return empty_indices, occupied_indices
    


    def compareCurrentToPreviousOccupancy(self):
        """
        Look through current state vs prev state and log indices (positions)
        where OCCUPANCY has changed
        """
        occupancy_change_list = []
        for i in range(8):
            for j in range(8):
                if self.current_state[i][j].occupancy != self.prev_state[i][j].occupancy:
                    occupancy_change_list.append((i, j)) #append tuple containing row and col of changed space
        return occupancy_change_list
    
    

    def determineIfCastlingRightsChanged(self, occupancy_change_list):
        """
        Checks occupancy list to see if a king or rook was moved off its original tile
        Refer to CastlingRights enum for reference on why these particular values are being subtracted
        original_rook_squares = [(0, 0), (0, 7), (7, 0), (7, 7)] # 4 board corners
        original_king_squares = [(0, 4), (7, 4)]
        """
        for tup in occupancy_change_list:
            if tup == (0, 0): # black queenside rook
                self.black_can_castle = CastlingRights(self.black_can_castle - 1)
            elif tup == (0, 7): # black kingside rook
                self.black_can_castle = CastlingRights(self.black_can_castle - 2)
            elif tup == (7, 0): # white queenside rook
                self.white_can_castle = CastlingRights(self.white_can_castle - 1)
            elif tup == (7, 7): # white kingside rook
                self.white_can_castle = CastlingRights(self.white_can_castle - 2)
            elif tup == (0, 4): # black king was moved
                self.black_can_castle = CastlingRights.NONE
            elif tup == (7, 4):
                self.white_can_castle = CastlingRights.NONE
    


    def detectCastle(self, occupancy_change_list):
        """
        Returns true if castling just occurred
        """
        moved_pieces = [self.prev_state[tup[0]][tup[1]].piece_type for tup in occupancy_change_list]
        if PieceType.KING in moved_pieces and PieceType.ROOK in moved_pieces and len(occupancy_change_list) == 4: 
            # if both the king and rook were moved at once, and two squares were left empty, it has to be a castle
            return True
        else:
            return False
        

    def determineCastleTypeAndUpdateState(self, occupancy_change_list):
        moved_colors = [self.prev_state[tup[0]][tup[1]].occupancy for tup in occupancy_change_list]
        print(moved_colors)
        
        if Occupancy.BLACK in moved_colors and Occupancy.WHITE in moved_colors:
            print('Error: black and white pieces should not be involved in one castle move!')
        
        elif Occupancy.WHITE in moved_colors: # white castled
            if (7, 0) in occupancy_change_list and (7, 4) in occupancy_change_list: # left rook was involved
                if self.white_can_castle != CastlingRights.BOTH and self.white_can_castle != CastlingRights.QUEENSIDE:
                    print(f'White has no right to castle queenside in this position!')
                self.current_state[7][0].piece_type = None
                self.current_state[7][0].occupancy = Occupancy.EMPTY
                self.current_state[7][2].piece_type = PieceType.KING
                self.current_state[7][2].occupancy = Occupancy.WHITE
                self.current_state[7][3].piece_type = PieceType.ROOK
                self.current_state[7][3].occupancy = Occupancy.WHITE
                self.current_state[7][4].piece_type = None
                self.current_state[7][4].occupancy = Occupancy.EMPTY
            elif (7, 7) in occupancy_change_list and (7, 4) in occupancy_change_list: # right rook was involved
                if self.white_can_castle != CastlingRights.BOTH or self.white_can_castle != CastlingRights.KINGSIDE:
                    print(f'White has no right to castle kingside in this position!')
                self.current_state[7][4].piece_type = None
                self.current_state[7][4].occupancy = Occupancy.EMPTY
                self.current_state[7][5].piece_type = PieceType.ROOK
                self.current_state[7][5].occupancy = Occupancy.WHITE
                self.current_state[7][6].piece_type = PieceType.KING
                self.current_state[7][6].occupancy = Occupancy.WHITE
                self.current_state[7][7].piece_type = None
                self.current_state[7][7].occupancy = Occupancy.EMPTY
            else:
                print(f'Error: Apparently no white rook was involved in castling')
        
        elif Occupancy.BLACK in moved_colors: # black castled
            if (0, 0) in occupancy_change_list and (0, 4) in occupancy_change_list:
                if self.black_can_castle != CastlingRights.BOTH and self.black_can_castle != CastlingRights.QUEENSIDE:
                    print(f'Black has no right to castle queenside in this position!')
                self.current_state[0][0].piece_type = None
                self.current_state[0][0].occupancy = Occupancy.EMPTY
                self.current_state[0][2].piece_type = PieceType.KING
                self.current_state[0][2].occupancy = Occupancy.WHITE
                self.current_state[0][3].piece_type = PieceType.ROOK
                self.current_state[0][3].occupancy = Occupancy.WHITE
                self.current_state[0][4].piece_type = None
                self.current_state[0][4].occupancy = Occupancy.EMPTY
            elif (0, 7) in occupancy_change_list and (0, 4) in occupancy_change_list:
                if self.black_can_castle != CastlingRights.BOTH and self.black_can_castle != CastlingRights.QUEENSIDE:
                    print(f'Black has no right to castle kingside in this position!')
                self.current_state[0][4].piece_type = None
                self.current_state[0][4].occupancy = Occupancy.EMPTY
                self.current_state[0][5].piece_type = PieceType.ROOK
                self.current_state[0][5].occupancy = Occupancy.WHITE
                self.current_state[0][6].piece_type = PieceType.KING
                self.current_state[0][6].occupancy = Occupancy.WHITE
                self.current_state[0][7].piece_type = None
                self.current_state[0][7].occupancy = Occupancy.EMPTY

        else:
            print('determineCastleTypeAndUpdateState(): all occupancies returned EMPTY somehow')
 

    def detectEnPassant(self, occupancy_change_list):
        return False



    def analyzeOccupancyChanges(self):
        """
        Determine new board state given only the previous board state & new square occupancy
        This is what the robot has to do in real life
        """
        occupancy_change_list = self.compareCurrentToPreviousOccupancy()
        if len(occupancy_change_list) == 0:
            print(f'Detected 0 occupancy changes! Something is probably wrong.')
            return
        
        """
        There are several cases for occupancy changes
        case 1: 2 occupancy changes, means regular piece move or regular take
        case 2: 3 occupancy changes, can mean en passant or error
        case 3: 4 occupancy changes, can mean castle or error
        case 4: >4 occupancy changes, must be error by board detector
        """


        if self.detectCastle(occupancy_change_list):
            print("castle detected")
            self.determineCastleTypeAndUpdateState(occupancy_change_list)
            




        """
        empty_idx, non_empty_idx = self.findWhichIndicesAreEmpty(occupancy_change_list) #find which space a piece was moved from
        pieces_that_were_moved = []
        for idx in empty_idx:
            pieces_that_were_moved.append()

        print(piece_that_was_moved.piece_type)
        print(piece_that_was_moved.occupancy)

        self.current_state[non_empty_idx[0]][non_empty_idx[1]] = piece_that_was_moved #update
        """

        """
        Populate the rest of the board using previous board state
        now that we have taken care of the moved piece
        """
        for i in range(8):
            for j in range(8):
                if (i, j) not in occupancy_change_list:
                    self.current_state[i][j] = self.prev_state[i][j]
                else:
                    print(f'Skipping index ({i}, {j}) for boardstate copy')

    

    

    def injectTestCase(self):
        print("Injecting test case.")
        empty_indices = [(7,1), (7,2), (7,3), (7,5), (7,6)]
        for idx in empty_indices:
            self.current_state[idx[0]][idx[1]].piece_type = None
            self.current_state[idx[0]][idx[1]].occupancy = Occupancy.EMPTY



board = ChessBoard(playing_as="white")

"""
fen = board.toFEN(board.current_state)
if board.stockfish.is_fen_valid(fen):
    board.stockfish.set_fen_position(fen)
    move = board.stockfish.get_best_move()
    print(move)
else:
    print(f"Invalid fen string: {fen}")
"""

#
board.simulateMoves(["e8c8", "a8d8"])
board.analyzeOccupancyChanges()


game_board = display.start()
while True:
    display.check_for_quit()
    display.update(board.toFEN(board.current_state), game_board)

