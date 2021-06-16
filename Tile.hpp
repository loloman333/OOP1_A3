//----------------------------------------------------------------------------------------------------------------------
// Tile.hpp
//
// Tile class represents a tile on the Board, is a super class for other tiles and can have walls and players on it
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#ifndef A2_TILE_HPP
#define A2_TILE_HPP

#include <iostream>
#include <string>
#include <vector>

const std::string WALL = "█";
const size_t TILE_HEIGHT = 5;
const size_t TILE_WIDTH = 9;
const size_t DIRECTION_AMOUNT = 4;
const size_t ROTATION_AMOUNT = 4;
const size_t TOP_ROW = 0;
const size_t BOTTOM_ROW = 4;
const size_t LEFT_COLUMN = 1;
const size_t RIGHT_COLUMN = 7;
const size_t MAX_PLAYERS = 4;
const size_t STANDARD_POSITION = 3;
const size_t MAX_PLAYER_POSITION = 2;
const size_t WALL_SIZE = 4;
const size_t PLAYER_ROW = 3;

const size_t INDEX_WITHOUT_WALL = 3;
const size_t INDEX_WITH_WALL = 7;

// Enums
enum class TileType
{
  T,
  L,
  I,
  O,
  U,
  X
};
enum class Rotation
{
  DEG0 = 0,
  DEG90 = 1,
  DEG180 = 2,
  DEG270 = 3
};
enum class Direction
{
  TOP = 0,
  LEFT = 1,
  BOTTOM = 2,
  RIGHT = 3,
  UNDEFINED = 4
};

class Player;
enum class PlayerColor;

class Tile
{
private:

  TileType type_;
  Rotation rotation_;
  std::vector<Player*> players_;
  std::vector<Direction> walls_;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Prints the Tile into a string vector
  ///
  /// @param tile the vector to print the tile onto
  //
  void fillTileString(std::vector<std::string>& tile);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// adds the player color onto the tile string
  ///
  /// @param tile_vector the vector to print the player color onto
  //
  void addPlayersToTile(std::vector<std::string>& tile_vector);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// calculates the new direction based on the given rotation
  ///
  /// @param dir the direction that should be changed based on the rotation
  /// @param rot the rotation value that is added to the direction
  /// @return the calculated direction based on the rotation
  //
  Direction calcDirection(Direction dir, Rotation rot);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// checks if the current position on the tile is in the corner
  ///
  /// @param row the row index to check
  /// @param col the col index to check
  /// @return true if this is in the corner and false if it is not
  //
  bool isCorner(size_t row, size_t col);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// setter for the walls_ variable
  ///
  /// @param directions vector of directions where walls are on the tile
  //
  void setWalls(std::vector<Direction> directions);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// sets the walls for the tile according to it's type
  //
  void setWallsAccordingToTileType();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// rotates the Walls according to the rotation value of the tile
  ///
  /// @param old_rotation rotation the tile had before being updated
  //
  void updateWallsAccordingToRotation(Rotation old_rotation);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// rotates the Walls according to the rotation value of the tile if it wasn't rotated
  //
  void updateWallsAccordingToRotation();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// sets the Walls according to tile type and then rotates to match it's rotation
  //
  void generateWalls();

public:

  static const std::vector<Direction> all_directions_;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// the default tile constructor
  //
  Tile() = default;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// constructor with a given tile type
  ///
  /// @param type the tiletype for the new tile
  //
  Tile(TileType type);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// constructor with a given tile type and rotation
  ///
  /// @param type the tiletype for the new tile
  /// @param rotation the rotation in which the tile has been rotated
  //
  Tile(TileType type, Rotation rotation);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// the default tile destructor
  //
  virtual ~Tile() = default;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// copy constructor of the Tile class
  ///
  /// @param tile the tile to be copied
  //
  Tile(const Tile& tile) = delete;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// assignment operator of the Tile class
  ///
  /// @param tile the tile to be copied
  //
  Tile& operator=(const Tile& tile) = delete;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// getter for the tile_type_
  ///
  /// @returns the tile type of the tile
  //
  TileType getType();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// getter for the rotation
  ///
  /// @returns the rotation of the tile
  //
  Rotation getRotation();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// getter for players_
  ///
  /// @returns the vector of the players currently standing on the tile
  //
  std::vector<Player*> getPlayers();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// setter for the rotation
  ///
  /// @param r the new rotation of the tile
  //
  void setRotation(Rotation r);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// setter for the tile_type
  ///
  /// @param type the new type of the tile
  //
  void setType(TileType type);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// pure virtual function to get the string of the tile
  ///
  /// @returns the string vector of the tile
  //
  std::vector<std::string> virtual getTileString() = 0;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// generates the raw string of a tile with walls and the players standing on the tile
  ///
  /// @returns the string vector of the tile
  //
  std::vector<std::string> getRawTileString();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// getter for the degrees the tile has been turned
  ///
  /// @returns the amount the tile has been rotated in degrees
  //
  size_t getRotationValue();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// getter for the tile type as string
  ///
  /// @returns the tile type as string
  //
  std::string getTileTypeString();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// adds the given player into the player vector of the tile
  ///
  /// @param player the player to be moved into the player vector
  //
  void addPlayer(Player* player);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// removes the given player based on color from the tile
  ///
  /// @param player_color the color of the player that is removed
  //
  void removePlayer(std::string player_color);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// gets the player in the player vector based on it's color
  ///
  /// @param player_color the color of the player that is returned
  /// @return the pointer of the corresponding player
  //
  Player* getPlayer(PlayerColor player_color);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// checks if the tile has a treasure
  ///
  /// @returns true if the tile has a treasure false if not
  //
  bool virtual hasTreasure();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// checks if the tile has an item
  ///
  /// @returns true if the tile has an item false if not
  //
  bool virtual hasItem();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// checks if the tile has a wall in given direction
  ///
  /// @param direction to be checked
  /// @returns true if the tile has a wall in this direction false if not
  //
  bool isWallInDirection(Direction direction);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// prints the tile
  //
  void print();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// rotates the tile in the given direction
  ///
  /// @param direction to be rotated in
  //
  void rotate(Direction dir);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// checks if the player with the given color is on this tile
  ///
  /// @param color to be checked
  /// @returns true if the player with the given color is on the tile false if not
  //
  bool isPlayerColorOnTile(PlayerColor color);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// adds a wall in the given direction
  ///
  /// @param direction for the wall to be inserted
  /// @returns true if the wall was inserted and false if one already was there
  //
  bool addWallInDirection(Direction direction);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// removes a wall in the given direction
  ///
  /// @param direction for the wall to be removed
  //
  void removeWallInDirection(Direction direction);
};

#endif // A2_TILE_HPP