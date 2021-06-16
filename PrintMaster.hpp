//----------------------------------------------------------------------------------------------------------------------
// PrintMaster.hpp
//
// The PrintMaster Class manages all outputs that need to be made for a Game
// That means it comes with a bunch of string constants and functions to print them
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#ifndef A2_PRINTMASTER_HPP
#define A2_PRINTMASTER_HPP

#include <string>
#include <vector>

const std::string UI_WELCOME = "Welcome to the Wild OOP Labyrinth!!!";
const std::string UI_PLAYER_COUNT = "Player Count (2-4): ";
const std::string UI_WRONG_COUNT = "Wrong Input only a Number from 2 to 4 is allowed!";
const std::string UI_WIN_1 = "The Player ";
const std::string UI_WIN_2 = " has won the game!";
const std::string UI_CLEAR = "\x1b[2J";

const std::string UI_ARROW_RIGHT = "-->";
const std::string UI_ARROW_LEFT = "<--";
const std::string UI_DOWN_ARROWS = "V                 V                 V";
const std::string UI_UP_ARROWS = "Ʌ                 Ʌ                 Ʌ";
const std::string UI_ARROW_BASES = "|                 |                 |";
const std::string UI_COLUMN_LABELS = "        1        2        3        4        5        6        7    ";
const std::string UI_TREASURE_COUNTER = "Treasure: ";
const std::string UI_PLAYER_BASE = "Player ";

const std::string COMMAND_INVALID = "Invalid command: ";
const std::string COMMAND_INVALID_PARAMETER = "Invalid parameter: ";
const std::string COMMAND_WRONG_NUMBER_ARGUMENTS = "Wrong number of arguments!";
const std::string COMMAND_TAKES_NO_ARGUMENTS = "This command does not take any arguments!";
const std::string COMMAND_NOT_ALLOWED = " is currently not allowed";
const std::string IMPOSSIBLE_MOVE = "Impossible move!";
const std::string ALL_TREASURES_FOUND = "All Treasures found, return to your startfield to win!";
const std::string CURRENT_TREASURE = "Current Treasure: ";
const std::string TREASURE_NUMBER = " Nr.: ";
const std::string FREE_TILE = "Free tile:";
const std::string INVALID_POSITION = "Invalid Position!";

const std::string ROPE_CANT_MOVE = "You can't move because you are tied up";
const std::string UI_LADDER_PROMPT = "You can go in all directions, with your next move. Choose wisely!";
const std::string UI_WHERE_WALL = "Where do you want to build the wall? ";
const std::string UI_WALL_EXISTS = "Wall already exists!";
const std::string UI_WALL_BUILT = "Wall built";
const std::string UI_NO_ITEM = "No item collected";
const std::string ROPE_NO_PLAYER = "No player to tie up here.";
const std::string ROPE_TIED_1 = "Player ";
const std::string ROPE_TIED_2 = " tied up.";
const std::string UI_BOMB = "BOOOOOOOOOMMMMM!!!!";
const std::string UI_AI_NAME = " Team: Triochter Bande";
const std::string UI_INFO = "[INFO]";

const size_t UI_PLAYER_TITLE_OFFSET = 58;
const size_t UI_ARROW_OFFSET = 17;
const size_t UI_BOTTOM_ARROW_TIPS_AT = 1;
const size_t UI_TOP_ARROW_TIPS_AT = 2;
const size_t UI_TITLE_ROW_MOD = 2;
const size_t UI_ARROW_BASE_MOD = 3;
const size_t UI_FIRST_ROW_LABEL_INDEX = 2;
const size_t UI_UP_ARROWS_EXTRA_SPACE = 3;

class Game;
enum class Direction;

class PrintMaster
{

private:
  Game& game_;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Generates and returns the UI lines based on the current state of the game
  ///
  /// @return A vector of string with all the UI lines
  //
  std::vector<std::string> generateUILines();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Adds a player title according to the player_index to a line
  ///
  /// @param line The line which should be modified
  /// @param player_index the index of the player
  //
  void addPlayerTitlesToLine(std::string& line, size_t& player_index);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Adds a treasure counter according to the player_index to a line
  ///
  /// @param line The line which should be modified
  /// @param player_index the index of the player
  //
  void addTreasureCountersToLine(std::string& line, size_t& player_index);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Adds the bases of the arrows to a line
  ///
  /// @param line The line where the arrow bases should be added
  //
  void addArrowBasesToLine(std::string& line);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Adds the tips of he arrows to aline based on a direction
  ///
  /// @param line The line where the arrow tips should be added
  /// @param direction The direction in which the arrows should face
  //
  void addArrowTipsToLine(std::string& line, Direction direction);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Print the game board, without Player Information
  //
  void printBoard();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the right side of one line of the UI
  ///
  /// @param print_arrow A boolean if an arrow should be printed in this line
  //
  void printRightUI(bool print_arrow);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the lines of the tiles for a line index
  ///
  /// @param tile_strings A vector of tile strings
  /// @param print_arrow The current line index
  //
  void printTilesOfLine(std::vector<std::vector<std::string>>& tile_strings, size_t tile_line_index);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the left side of one line of the UI
  ///
  /// @param row_index index of the current row
  /// @param line_index index of the current line inside the row
  /// @param row_label_index index when to print the next row label
  /// @param print_arrow A boolean if an arrow should be printed in this line
  //
  void printLeftUI(size_t& row_index, size_t line_index, size_t& row_label_index, bool& print_arrow);

public:
  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Constructor for the Print Master
  /// Sets game_ based on parameter
  ///
  /// @param game The game instance to print for
  //
  PrintMaster(Game& game);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// destructor of the PrintMaster class
  //
  ~PrintMaster() = default;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// copy constructor of the PrintMaster class
  ///
  /// @param print_master the print_master to be copied
  //
  PrintMaster(const PrintMaster& print_master) = delete;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// assignment operator of the PrintMaster class
  ///
  /// @param print_master the print_master to be copied
  //
  PrintMaster& operator=(const PrintMaster& print_master) = delete;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Print the whole game with board, UI and everything that needs to be printed
  //
  void printGame();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the win message for the color passed
  ///
  /// @param color The color of the player that won
  //
  void printWin(std::string color);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Resets the UI by clearing the terminal and printing the game again
  //
  void resetUI();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the game if the "gameboard off" command wasn't entered
  //
  void printGameIfNecessary();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the current free tile
  //
  void printFreeTile();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the treasure the current player is looking for
  //
  void printTreasure();

  // Items

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the message that a player was tied up
  ///
  /// @param player The player that was tied up
  //
  void tiedUpPlayer(std::string player);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the message that a player is roped
  //
  void playerRoped();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the message that no player is here to be tied up
  //
  void noPlayerToTieUp();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the message that the ladder was used
  //
  void ladderUsed();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the question where the bricks should be placed
  //
  void whereWall();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the message that a wall already exists
  //
  void wallExists();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the message that the wall was buit
  //
  void wallBuilt();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the current player has no item to use
  //
  void noItem();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the message that the dynamite item was used
  //
  void printBomb();

  // Error messages

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the error message that a command is invalid
  ///
  /// @param command the command that is invalid as string
  //
  void invalidCommand(std::string command);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the error message that a parameter is invalid
  ///
  /// @param parameter the parameter that is invalid as string
  //
  void invalidParameter(std::string parameter);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the error message that the user entered a wrong number of arguments
  //
  void wrongNumberArguments();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the error message that a command takes no arguments
  //
  void commandTakesNoArguments();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the error message that a command is not allowed
  ///
  /// @param a vector of string with the command that os not alloweed and its paramteres
  //
  void commandNotAllowed(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the error message that a command is not allowed
  ///
  /// @param the command that is not allowed
  //
  void commandNotAllowed(std::string command);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the error message that a move is impossible
  //
  void impossibleMove();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the error message that the insert position is invalid
  //
  void invalidInsertPosition();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// prints AI name
  //
  void printAIName();
};

#endif // A2_PRINTMASTER_HPP
