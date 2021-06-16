//----------------------------------------------------------------------------------------------------------------------
// TreasureTile.hpp
//
// The TreasureTile class is a subclass of "Tile". It represents a treasure on
// the board.
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#ifndef A2_TREASURE_TILE_HPP
#define A2_TREASURE_TILE_HPP

#include "Tile.hpp"
#include <string>
#include <vector>

const size_t TREASURE_TILE_ROW = 2;
const size_t MAX_SINGLE_DIGIT = 9;
const std::string TREASURE_STRING_T = "T";
const std::string TREASURE_STRING_0 = "0";

const std::vector<size_t> TREASURE_DEG_0 = {1, 2, 5};
const std::vector<size_t> TREASURE_DEG_90 = {3, 4, 7};
const std::vector<size_t> TREASURE_DEG_180 = {8, 11, 12};
const std::vector<size_t> TREASURE_DEG_270 = {6, 9, 10};

class Treasure;

class TreasureTile : public Tile
{
private:
  Treasure* treasure_;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Creates string with treasure id for the tile string
  ///
  /// @return string with treasure id
  //
  std::string createTreasureId();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Calculates rotation of tile, depending on treasure id
  ///
  /// @param treasure_id of treasure
  ///
  /// @return Rotation calculated depending on treasure id
  //
  Rotation calculateRotation(size_t treasure_id);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Checks if given number has one or more digits
  ///
  /// @param number to check
  ///
  /// @return true if number has one digit
  //
  bool isSingleDigit(size_t number);

public:
  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Constructor for the TreasureTile class
  /// Takes a tresure pointer and sets TileType to T
  ///
  /// @param treasure for the TreasureTile
  //
  TreasureTile(Treasure* treasure);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Constructor for the TreasureTile class
  /// Takes a TileType and a Treasure pointer and sets variables
  ///
  /// @param type, TileType of the TreasureTile
  /// @param treasure, treasure for the TreasureTile
  //
  TreasureTile(TileType type, Treasure* treasure);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// destructor of the TreasureTile class
  //
  ~TreasureTile() = default;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// copy constructor of the TreasureTile class
  ///
  /// @param treasure_tile the treasure_tile to be copied
  //
  TreasureTile(const TreasureTile& treasure_tile) = delete;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// assignment operator of the TreasureTile class
  ///
  /// @param treasure_tile the treasure_tile to be copied
  //
  TreasureTile& operator=(const TreasureTile& treasure_tile) = delete;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Virtual function of Tile.hpp
  ///
  /// @return vector<string> which represents the tile
  //
  std::vector<std::string> getTileString();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// If necessary adds treasure id to the tile string
  ///
  /// @param tile_string reference
  //
  void addTreasureIdToTile(std::vector<std::string>& tile_string);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Virtual function from Tile.hpp
  /// Checks if tile has a Treasure
  ///
  /// @return true if there is a treasure on the Tile and is not collected yet
  //
  bool hasTreasure();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Marks treasure as found
  //
  void foundTreasure();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for treasure
  ///
  /// @return treasure pointer of treasure on tile
  //
  Treasure* getTreasure();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for found
  ///
  /// @return true if treasure is marked as found
  //
  bool getFound();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Setter for treasure
  ///
  /// @param treasure, sets treasure of tile
  //
  void setTreasure(Treasure* treasure);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Setter for found
  ///
  /// @param collected, sets collected of treasure
  //
  void setFound(bool collected);
};

#endif // A2_TREASURE_TILE_HPP
