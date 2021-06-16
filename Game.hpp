//----------------------------------------------------------------------------------------------------------------------
// Game.hpp
//
// The Game class is the main class of this project.
// It implements the logic and controlls the flow of the game.
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#ifndef A2_GAME_HPP
#define A2_GAME_HPP

#include <string>
#include <vector>

const size_t BOARD_SIZE = 7;
const size_t EXIT_DEFAULT = 0;
const size_t EXIT_MEMORY_ERROR = 1;
const size_t PLAYER_COUNT_INDEX = 0;

const size_t NORMAL_GAME_NORMAL_L_TILES = 11;
const size_t NORMAL_GAME_NORMAL_I_TILES = 11;
const size_t NORMAL_GAME_NORMAL_T_TILES = 12;

const size_t BONUS_GAME_NORMAL_L_TILES = 9;
const size_t BONUS_GAME_NORMAL_I_TILES = 9;
const size_t BONUS_GAME_NORMAL_T_TILES = 12;

const std::string TWO_PLAYERS = "2";
const std::string THREE_PLAYERS = "3";
const std::string FOUR_PLAYERS = "4";
const std::string PROMPT = " > ";

class Tile;
class Player;
class Treasure;
class CommandMaster;
class PrintMaster;
class AIMaster;
enum class PlayerColor;
enum class TileType;
enum class Direction;

class Game
{
private:
  CommandMaster* command_master_;
  PrintMaster* print_master_;
  AIMaster* ai_master_;


  std::vector<std::vector<Tile*>> board_;
  std::vector<Treasure*> treasures_;
  std::vector<Player*> players_;
  Tile* free_tile_;
  size_t current_player_index_ = 0;
  bool quit_ = false;
  bool bonus_items_ = false;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Constructor for the Game class
  /// Private for singleton
  //
  Game();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Copy constructor for the game class
  /// Private for singleton
  //
  Game(const Game&);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Operator = for the game class
  /// Private for singleton
  ///
  /// @param Game reference
  ///
  /// @return Game reference
  //
  Game& operator=(const Game&);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Checks if the game is still running
  ///
  /// @return true if game is running, false if not
  //
  bool gameIsRunning();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Creates new player with specific color and add
  /// it to the players vector
  ///
  /// @param player color
  //
  void addPlayer(PlayerColor color);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Game preparations
  ///
  //--------------------------------------------------------------------------------------------------------------------

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Creates all treasures and pushes them to the treasures vector
  //
  void fillTreasures();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Starts the game, asks for the player count
  //
  void gameStart();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Plays one round
  //
  void playRound();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Randomly fills the board with all tiles
  //
  void fillBoard();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Distributes all treasures to all players
  //
  void distributeTreasures();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Helper function for fill board
  /// Fills all static tiles to the board
  ///
  /// @param treasure_index current treasure index
  //
  void fillStaticTiles(size_t& treasure_index);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Helper function for fill board
  /// Fills all variable tiles to the board
  ///
  /// @param treasure_index current treasure index
  //
  void fillVariableTiles(size_t& treasure_index);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Helper function for fill board
  /// Adds all items to the vector
  ///
  /// @param Tile vector reference where to add items
  //
  void addNewItemTilesToVector(std::vector<Tile*>& vector);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Helper function for fill board
  /// Adds all normal Tiles to the vector
  ///
  /// @param Tile vector reference where to add normal tiles
  /// @param TileType of Tile
  /// @param count, how often should this tile be created
  //
  void addNewNormalTilesToVector(std::vector<Tile*>& vector, TileType type, size_t count);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Helper function for fill board
  /// Adds all Treasure Tiles to the vector
  ///
  /// @param Tile vector reference where to add normal tiles
  /// @param TileType of Tile
  /// @param count, how often should this tile be created
  /// @param treasure_index, index of treasure
  //
  void addNewTreasureTilesToVector(std::vector<Tile*>& vector, TileType type, size_t count, size_t& treasure_index);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Checks if given row, col is in a moveabline line on the board
  ///
  /// @param row_index
  /// @param col_index
  ///
  /// @return true if the coordinates are in moveable line
  //
  bool isInMoveableLine(size_t row_index, size_t col_index);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Free memory
  ///
  //--------------------------------------------------------------------------------------------------------------------

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Deletes all Treasures on heap
  //
  void deleteTreasures();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Deletes all Free Tiles on heap
  //
  void deleteFreeTile();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Deletes board on heap
  //
  void deleteBoard();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Deletes items from players on heap
  //
  void deleteItemsFromPlayers();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Deletes all players on heap
  //
  void deletePlayers();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Deletes tile on heap
  ///
  /// @param Tile pointer to delete
  //
  void deleteTile(Tile* tile);

public:
  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Singleton
  /// Returns a static instance of Game Class
  ///
  /// @return static Game instance
  //
  static Game& instance();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Destructor of Game Class
  //
  ~Game();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Starts the Game
  /// Do all Game preparations, play rounds till someone wins or quits
  ///
  /// @return exit status
  //
  int run();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Stops the game
  //
  void quitGame();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// increment current player index
  //
  void nextPlayer();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// checks if given row, col is a corner on board
  ///
  /// @param row_index
  /// @param col_index
  /// @return true if its a corner
  //
  bool isCorner(size_t row_index, size_t col_index);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for CommandMaster
  ///
  /// @return CommandMaster pointer
  //
  CommandMaster* getCommandMaster();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for PrintMaster
  ///
  /// @return PrintMaster pointer
  //
  PrintMaster* getPrintMaster();

  // TODO
  AIMaster* getAIMaster();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for board
  ///
  /// @return board reference
  //
  std::vector<std::vector<Tile*>>& getBoard();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for players
  ///
  /// @return players reference
  //
  std::vector<Player*>& getPlayers();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for treasures
  ///
  /// @return treasures reference
  //
  std::vector<Treasure*>& getTreasures();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for current player
  ///
  /// @return player pointer of current player
  //
  Player* getCurrentPlayer();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for free tile
  ///
  /// @return Tile pointer of free Tile
  //
  Tile* getFreeTile();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for bonus flag
  ///
  /// @return true if its a bonus game
  //
  bool getBonusItemsFlag();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Setter for free tile
  ///
  /// @param Tile pointer, sets free tile
  //
  void setFreeTile(Tile* tile);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Setter bonus game
  ///
  /// @param choice, true if you want to play a game with bonus items
  //
  void setBonus(bool choice);
};

#endif // A2_GAME_HPP
