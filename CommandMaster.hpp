//----------------------------------------------------------------------------------------------------------------------
// CommandMaster.hpp
//
// The CommandMaster handles all commands entered in the command line
// and calls the corresponding functions in other classes
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#ifndef A2_COMMANDMASTER_HPP
#define A2_COMMANDMASTER_HPP

#include <string>
#include <vector>

const std::vector<std::string> PLAYER_MOVEMENT{"w", "\x1b[A", "a", "\x1b[D", "s", "\x1b[B", "d", "\x1b[C", "go"};

const size_t MOVE_TOP = 2;
const size_t MOVE_LEFT = 4;
const size_t MOVE_BOTTOM = 6;
const size_t MOVE_RIGHT = 8;

const size_t ARROW_UP_INDEX = 1;
const size_t ARROW_LEFT_INDEX = 3;
const size_t ARROW_DOWN_INDEX = 5;
const size_t ARROW_RIGHT_INDEX = 7;

const size_t MOVEMENT_DIRECTION_INDEX = 1;

const size_t COMMAND_INDEX = 0;
const size_t FIRST_ARGUMENT_INDEX = 1;
const size_t SECOND_ARGUMENT_INDEX = 2;

class Player;
class Tile;
class Treasure;
class Game;
class Item;
enum class Direction;

class CommandMaster
{

private:
  Game& game_;

  bool inserted_ = false;
  bool show_treasure_ = false;
  bool show_gamefield_ = true;

  std::string last_insert_direction_ = "";
  size_t last_insert_row_col_ = 0;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// takes the tokens of the command and executes the corresponding command
  ///
  /// @param tokens the command tokens that have to be handled
  /// @returns true if the game should end else false
  //
  bool executeCommand(std::vector<std::string>& tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// takes a string and a number reference and converts the given string into a size_t
  ///
  /// @param token the string to be converted
  /// @param number the number that saves the converted string
  /// @returns true if the string could be converted else false
  //
  bool stringToSizeT(std::string token, size_t& number);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// toggles printing the treasure name and id on
  /// and prints the gameboard if not turned off
  ///
  /// @param tokens the command tokens that have to be handled
  //
  void showTreasure(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// toggles the printing of the treasure off and clears the commandline
  ///
  /// @param tokens the command tokens that have to be handled
  //
  void hideTreasure(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// prints the free tile
  ///
  /// @param tokens the command tokens that have to be handled
  //
  void showFreeTile(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// rotates the free tile in given direction
  ///
  /// @param tokens the command tokens that have to be handled
  //
  void rotateFreeTile(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// toggles the printign of gamefield on/off or prints the gamefield depending on user input
  ///
  /// @param tokens the command tokens that have to be handled
  //
  void gameField(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// ends the current player's turn and checks if the game is won
  ///
  /// @param tokens the command tokens that have to be handled
  /// @returns true if the command is executed properly else false
  //
  bool finish(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// checks if the player needs the treasure he is standing on
  ///
  /// @param treasure pointer to the treasure the player is standing on
  /// @returns true if the player needs the treasure else false
  //
  bool currentPlayerNeedsTreasure(Treasure* treasure);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// collects the treasure the player is standing on and updates the found treasures for the player
  ///
  /// @param treasure pointer to the treasure the player is standing on
  //
  void currentPlayerCollectTreasure(Treasure* treasure);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// collects the item the player is standing on
  ///
  /// @param item pointer to the item the player is standing on
  //
  void currentPlayerCollectItem(Item* item);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// checks if the current tile has a treasure and if the player needs it and sets the treasure reference to it
  ///
  /// @param treasure_to_collect reference to the treasure the player is standing on
  /// @returns true if the treasure should be collected else false
  //
  bool checkCollectTreasure(Treasure*& treasure_to_collect);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// checks if the current tile has an item and if the player can pick up an item
  ///
  /// @param item_to_collect reference to the item the player is standing on
  /// @returns true if the item should be collected else false
  //
  bool checkCollectItem(Item*& item_to_collect);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// checks if the current player has won the game
  ///
  /// @returns true if the player has won else false
  //
  bool checkWin();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// inserts the free tile where the command tells it to and checks for invalid inputs
  ///
  /// @param tokens the command tokens that have to be handled
  //
  void insert(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// checks where the free tile should be placed and inserts it
  ///
  /// @param tokens the command tokens that have to be handled
  //
  void insertTile(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// moves the whole row one tile ahead and then places the free tile on the empty place
  ///
  /// @param tokens the command tokens that have to be handled
  //
  void insertRow(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// moves the whole column one tile ahead and then places the free tile on the empty place
  ///
  /// @param tokens the command tokens that have to be handled
  //
  void insertColumn(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// checks if the inert parameters are valid
  ///
  /// @param tokens the command tokens that have to be handled
  /// @returns true if the parameters are valid else false
  //
  bool checkInsertParameter(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// checks if the given row or column is movable or not
  ///
  /// @param row_col the row/column index that should be checked
  /// @returns true if the row/column is moveable else false
  //
  bool isInMoveableRowOrCol(size_t row_col);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// checks if the given insert direction is valid
  ///
  /// @param direction the direction string that is checked
  /// @returns true if the direction is valid else false
  //
  bool isValidInsertDirection(std::string direction);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// checks if the new insert doesn't revert the last input
  ///
  /// @param tokens the command tokens that have to be handled
  /// @returns false if the new insert reverts the last insert else true
  //
  bool checkLastInsert(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// compares the last insert to the new insert
  ///
  /// @param direction the direction that is compared
  /// @param alias the alias of thecommand that is compared
  /// @param row_col row/col that is compared
  /// @returns false if the new insert reverts the old insert else true
  //
  bool compareLastInsert(std::string direction, std::string alias, std::string row_col);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// updates the players coordinates on the field if they have been moved with the insert
  ///
  /// @param players the players that have been moved with a tile
  /// @param row the new row of the tile
  /// @param column the new column of the tile
  //
  void playersUpdateRowColumn(std::vector<Player*> players, size_t row, size_t column);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// moves a player onto another Tile
  ///
  /// @param from the tile the players need to be removed from
  /// @param row the row of the new tile
  /// @param column the column of the new tile
  //
  void movePlayersToTile(Tile* from, size_t row, size_t column);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// moves the current player to the location given in the command tokens
  ///
  /// @param tokens the command tokens that have to be handled
  //
  void movePlayer(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// checks if the command tokens of the movement command are corrent
  ///
  /// @param tokens the command tokens that have to be handled
  /// @returns true if the input is correct else false
  //
  bool checkMoveInput(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// gets the direction to be moved from the command tokens
  ///
  /// @param tokens the command tokens that have to be handled
  /// @returns the direction the move has to go
  //
  Direction getDirection(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// checks if the command tokens of the movement command are corrent
  ///
  /// @param tokens the command tokens that have to be handled
  /// @returns the amount to be moved
  //
  int getAmount(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// calculates the modifier on the row and column that have to be added to move in specified direction
  /// row_modifier = 0 if the movement is in column direction only
  /// else it is 1 or -1 depending on moving forwards or backwards
  /// col_modifier = 0 if the movement is in row direction only
  /// else it is 1 or -1 depending on moving forwards or backwards
  ///
  /// @param direction the direction in which the move happens
  /// @param row_modifier the modifier that is added to the movement to walk in specified direction
  /// @param col_modifier the modifier that is added to the movement to walk in specified direction
  //
  void getMovementModifier(Direction direction, int& row_modifier, int& col_modifier);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// checks if the movement is possible or if walls or the edge are in the way
  ///
  /// @param direction the direction to be moved in
  /// @param movement the amount of tiles to be moved
  /// @param row_modifier the modifier to be used when moving in row direction
  /// @param col_modifier the modifier to be used when moving in column direction
  /// @returns true if the move is possible else false
  //
  bool isMovePossible(Direction direction, size_t movement, int& row_modifier, int& col_modifier);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// moves the player in the specified direction
  ///
  /// @param player the player to be moved
  /// @param movement the amount of tiles to be moved
  /// @param row_modifier the modifier to be used when moving in row direction
  /// @param col_modifier the modifier to be used when moving in column direction
  //
  void moveInDirection(Player* player, size_t movement, int& row_modifier, int& col_modifier);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// prints the error Message that the move is not allowed
  ///
  /// @param command the command string that is not allowed and needs to be printed
  //
  void moveNotAllowed(std::string command);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// uses the item of the current player, if the player has none an error is printed
  //
  void useItem(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// activates the AI to play this round
  //
  void play(std::vector<std::string> tokens);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// outputs the AI name
  //
  void whoAmI(std::vector<std::string> tokens);

  //TODO
  void checkPathFinding(std::vector<std::string> tokens);
  void teleportPlayer(Player* current_player, size_t to_row, size_t to_column);

public:
  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// constructor with a reference to the game
  ///
  /// @param game the game on which the CommandMaster operstes
  //
  CommandMaster(Game& game);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// destructor of the CommandMaster class
  //
  ~CommandMaster() = default;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// copy constructor of the CommandMaster class
  ///
  /// @param command_master the command_master to be copied
  //
  CommandMaster(const CommandMaster& command_master) = delete;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// assignment operator of the CommandMaster class
  ///
  /// @param command_master the command_master to be copied
  //
  CommandMaster& operator=(const CommandMaster& command_master) = delete;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// tokenize takes the string input and splits it up into tokens for the other commands to operate with
  ///
  /// @param input the string of the whole command
  //
  std::vector<std::string> tokenize(std::string input);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// handles all command  the user inputs on his turn and returns if the game should end or not
  ///
  /// @returns true if the game should be ended else false
  //
  bool handleCommand();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Checks if given string only contains numbers
  /// https://stackoverflow.com/questions/8888748/how-to-check-if-given-c-string-or-char-contains-only-digits
  ///
  /// @param string to check
  ///
  /// @return true if only contains numbers
  //
  bool is_digits(const std::string& str);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// resets the round so round specific flags are on their standard values
  //
  void reset();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for the showTreasure variable
  ///
  /// @returns the value of showTreasure
  //
  bool getShowTreasure();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for the showGamefield variable
  ///
  /// @returns the value of showGamefield
  //
  bool getShowGamefield();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// gets the direction opposite of the one given
  ///
  /// @param direction the direction from which the opposite is needed
  /// @returns the direction in the opposite direction
  //
  Direction getOppositeDirection(Direction direction);
};

#endif // A2_COMMANDMASTER_HPP
