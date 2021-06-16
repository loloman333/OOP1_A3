//----------------------------------------------------------------------------------------------------------------------
// Item.hpp
//
// The Item class represent an item that can be collected and used by players
// Implements the functionality for using items
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#ifndef A2_ITEM_HPP
#define A2_ITEM_HPP

#include <string>
#include <vector>

const std::vector<std::string> WALL_VALID_RESPONSES{"top", "left", "bottom", "right"};

class Game;
class Tile;
enum class Direction;

enum class ItemType
{
  ROPE,
  DYNAMITE,
  BRICKS,
  LADDER
};

class Item
{
private:
  ItemType item_type_;
  bool found_ = false;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Uses the brick item for the current player
  ///
  /// @param game A reference to the game object
  //
  void useBricks(Game& game);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Uses the dynamite item for the current player
  ///
  /// @param game A reference to the game object
  //
  void useDynamite(Game& game);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Uses the ladder item for the current player
  ///
  /// @param game A reference to the game object
  //
  void useLadder(Game& game);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Uses the rope item for the current player
  ///
  /// @param game A reference to the game object
  //
  void useRope(Game& game);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Checks if a string is in the list of valid responses
  ///
  /// @param response the string to check
  /// @return A boolean if a string is in the list of valid responses
  //
  bool isValidResponse(std::string response);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Takes a string and return the corresponding Direction
  ///
  /// @param string the string to "convert" into a Direction
  /// @return A Direction corresponding to the string
  //
  Direction getDirectionFromString(std::string string);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Sets a Wall on the given tile in the given direction
  ///
  /// @param direction the direction to set the wall in
  /// @param tile A pointer to the tile to set the wall on
  //
  void setBricksInDirection(Direction direction, Tile* tile);

public:
  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Constructor for the Item class
  /// Sets item_type_ based on Parameter
  ///
  /// @param item_type_ The type of the item
  //
  Item(ItemType item_type);

  //-------------------------------------------------------------------------------------------------------------------
  ///
  /// destructor of the Item class
  //
  ~Item() = default;

  //-------------------------------------------------------------------------------------------------------------------
  ///
  /// copy constructor of the Item class
  ///
  /// @param item the item to be copied
  //
  Item(const Item& item) = default;

  //-------------------------------------------------------------------------------------------------------------------
  ///
  /// assignment operator of the Item class
  ///
  /// @param item the item to be copied
  //
  Item& operator=(const Item& item) = default;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// A getter for item_type_
  ///
  /// @return The type of the item
  //
  ItemType getItemType();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Returns the type of the item as three-charcter-string in caps
  ///
  /// @return The item type as string
  //
  std::string getItemTypeAsString();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// A getter for found_
  ///
  /// @return A boolean if the item is currently found
  //
  bool getFound();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Setter for found_
  ///
  /// @param found A boolean if the item is currently found
  //
  void setFound(bool found);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Uses this item for the current player
  //
  void use();
};

#endif // A2_ITEM_HPP