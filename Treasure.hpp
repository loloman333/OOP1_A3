//----------------------------------------------------------------------------------------------------------------------
// Treasures.hpp
//
// The treasure class represent a Treasure.
// Players need to collect these treasures to win the game.
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#ifndef A2_TREASURE_HPP
#define A2_TREASURE_HPP

#include <string>
#include <vector>

const std::vector<std::string> TREASURE_NAMES{
    "Totenkopf",     "Schwert",       "Goldsack",    "Schlüsselbund", "Sombrero",   "Ritterhelm", "Buch",     "Krone",
    "Schatztruhe",   "Kerzenleuchte", "Schatzkarte", "Goldring",      "Eule",       "Hofnarr",    "Eidechse", "Käfer",
    "Flaschengeist", "Kobold",        "Schlange",    "Geist",         "Fledermaus", "Spinne",     "Ratte",    "Motte"};

class Treasure
{
private:
  std::string name_;
  size_t treasure_id_;
  bool found_ = false;

public:
  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Constructor for the Treasure class
  /// Sets name_ and treasure_id_ of the Treasure
  ///
  /// @param name of the treasure
  /// @param treasure_id of the treasure
  //
  Treasure(std::string name, size_t treasure_id);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// destructor of the Treasure class
  //
  ~Treasure() = default;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// copy constructor of the Treasure class
  ///
  /// @param treasure the treasure to be copied
  //
  Treasure(const Treasure& treasure) = delete;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// assignment operator of the Treasure class
  ///
  /// @param treasure the treasure to be copied
  //
  Treasure& operator=(const Treasure& treasure) = delete;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for name_
  ///
  /// @return name of the Treasure
  //
  std::string getName();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for treasure_id_
  ///
  /// @return treasure_id of the Treasure
  //
  size_t getTreasureId();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for found_
  ///
  /// @return found, indicates whether the treasure was collected
  //
  bool getFound();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Setter for found_
  ///
  /// @param found, indicates whether the treasure was collected
  //
  void setFound(bool found);
};

#endif // A2_TREASURE_HPP
