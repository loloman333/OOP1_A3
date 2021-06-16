//----------------------------------------------------------------------------------------------------------------------
// Player.hpp
//
// The Player class represents a Player on the board.
// It has several variables getter and setter for a player.
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#ifndef A2_PLAYER_HPP
#define A2_PLAYER_HPP

#include <string>
#include <vector>

class Treasure;
class Item;

enum class PlayerColor
{
  RED = 'R',
  YELLOW = 'Y',
  BLUE = 'B',
  GREEN = 'G'
};
class Player
{
private:
  std::vector<Treasure*> covered_stack_;
  size_t nr_found_treasures_;
  PlayerColor player_color_;
  size_t row_;
  size_t col_;
  Item* item_ = nullptr;
  bool using_ladder_ = false;
  bool tied_ = false;

public:
  static const std::vector<PlayerColor> player_colors_;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Constructor for the Player class
  /// Sets nr_found_treasures_ to default value and player_color_ based on Parameter
  ///
  /// @param player_color The color that the player should represent
  //
  Player(PlayerColor player_color);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// destructor of the Player class
  //
  ~Player() = default;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// copy constructor of the Player class
  ///
  /// @param player the player to be copied
  //
  Player(const Player& player) = delete;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// assignment operator of the Player class
  ///
  /// @param player the player to be copied
  //
  Player& operator=(const Player& player) = delete;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// A getter for covered_stack_
  /// Returns a reference!
  ///
  /// @return A reference to the covered_stack_ vector of treasure pointers
  //
  std::vector<Treasure*>& getCoveredStackRef();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for nr_found_treasures
  ///
  /// @return Number of treasures already found by this player
  //
  size_t getNrFoundTreasures();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for player_color_
  ///
  /// @return The color the player represents
  //
  PlayerColor getPlayerColor();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Returns the color of the player as string starting with an uppercase letter
  ///
  /// @return The players color as string
  //
  std::string getPlayerColorAsString();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for row_
  ///
  /// @return The row the player is currently standing in
  //
  size_t getRow();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for col_
  ///
  /// @return The column the player is currently standing in
  //
  size_t getCol();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for item_
  ///
  /// @return A pointer to the item the player is currently holding
  //
  Item* getItem();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for using_ladder_
  ///
  /// @return A boolean if the player is currently using the ladder item
  //
  bool getUsingLadder();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for tied_
  ///
  /// @return A boolean if the player is tied up by the rope item
  //
  bool getTied();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Setter for nr_found_treasures_
  ///
  /// @param nr_found_treasures New Number of treasures already found by this player
  //
  void setNrFoundTreasures(size_t nr_found_treasures);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Setter for row_
  ///
  /// @param row New row of the player
  //
  void setRow(size_t row);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Setter for col_
  ///
  /// @param col New column of the player
  //
  void setCol(size_t col);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Setter for row_ and col_
  ///
  /// @param row New row of the player
  /// @param col New column of the player
  //
  void setRowCol(size_t row, size_t column);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Setter for item_
  ///
  /// @param item The pointer to the new item of the player
  //
  void setItem(Item* item);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Setter for using_ladder_
  ///
  /// @param using_ladder new flag if the player is currently using the ladder item
  //
  void setUsingLadder(bool using_ladder);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Setter for tied_
  ///
  /// @param tied new flag if the player is tied up by the rope item
  //
  void setTied(bool tied);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Increments the Number of treasures found by this player by 1
  //
  void incrementNrFoundTreasures();
};

#endif // A2_PLAYER_HPP
