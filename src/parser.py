from pydantic import BaseModel, Field
from enum import Enum
import pytest
import heapq


class ZoneType(Enum):
    NORMAL = "normal"
    BLOCKED = "blocked"
    RESTRICTED = "restricted"
    PRIORITY = "priority"


class HubMetadata(BaseModel):
    zone: ZoneType = ZoneType.NORMAL
    color: str | None = None
    max_drones: int = 1


class HubConn(BaseModel):
    to: str
    max_link_cap: int = 1


class Connection(BaseModel):
    h1: str
    h2: str
    max_link_cap: int = 1

    @classmethod
    def _parse_connection(cls, conn: str, line: int) -> "Connection":
        ls = conn.split(maxsplit=1)
        if not 1 <= len(ls) <= 2:
            raise Exception(
                f"Error: Line #{line} -> expecting shape 'hub1-hub2 [optional_metadata]' got {conn}"
            )
        try:
            hub1, hub2 = ls[0].split("-")
        except Exception:
            raise Exception(
                f"Error: Line #{line} -> expecting shape 'hub1-hub2 [optional_metadata]' got {conn}"
            )
        if len(ls) == 1:
            return Connection(h1=hub1, h2=hub2)
        try:
            if not (ls[1].startswith("[") and ls[1].endswith("]")):
                raise Exception(
                    f"Error: Line #{line} -> metadata key=value pairs must be enclosed by []"
                )
            metadata = ls[1][1:-1].split("=", 1)
            if metadata[0].strip() != "max_link_capacity":
                raise Exception(
                    f"Error: Line #{line} -> invalid metadata, expecting [max_link_capacity=int] got {ls[1]}"
                )
            max_link_cap = int(metadata[1])
            if max_link_cap <= 0:
                raise Exception(
                    f"Error: Line #{line} -> max capacity value must be positive int, got {ls[1]}"
                )
        except ValueError:
            raise Exception(
                f"Error: Line #{line} -> max capacitiy value must be positive int, got {ls[1]}"
            )
        return Connection(h1=hub1, h2=hub2, max_link_cap=max_link_cap)


class Hub(BaseModel):
    name: str
    x: int
    y: int
    conns: dict[str, Connection] = {}
    metadata: HubMetadata

    @classmethod
    def _parse_metadata(cls, meta: str, line: int) -> HubMetadata:
        allowed_keys = set(["zone", "color", "max_drones"])
        if not (meta.startswith("[") and meta.endswith("]")):
            raise Exception(
                f"Error: Line #{line} -> malformed metadata, key value pairs must be enclosed by []"
            )
        met = meta[1:-1].split()
        try:
            cleaned = [tuple(m.split("=", 1)) for m in met]
        except Exception:
            raise Exception(
                f"Error: Line #{line} -> expected [key=val,..] for metadata, got {meta}"
            )
        mid = {}
        for item in cleaned:
            if len(item) != 2:
                raise Exception(
                    f"Error: Line #{line} -> expecting key=value format got {item}"
                )
            if item[0] in mid:
                raise Exception(f"Error: Line #{line} -> duplicate key for {item[0]}")
            if item[0] not in allowed_keys:
                raise Exception(
                    f"Error: Line #{line} -> {item[0]} not in allowed metadata fields ({allowed_keys})"
                )
            if item[0] == "zone":
                try:
                    mid[item[0]] = ZoneType(item[1])
                    continue
                except Exception:
                    raise Exception(
                        f"Error: Line #{line} -> {item[1]} it not a valid Zone Type ('normal', 'blocked', 'restricted', 'priority')"
                    )
            if item[0] == "max_drones":
                try:
                    val = int(item[1])
                    if val <= 0:
                        raise Exception(
                            f"Error: Line #{line} -> {item[1]} is not a valid max_drones value (must be positive int)"
                        )
                    mid[item[0]] = val
                    continue
                except ValueError:
                    raise Exception(
                        f"Error: Line #{line} -> max_drones must be a positive int, got {item[1]}"
                    )
            mid[item[0]] = item[1]
        return HubMetadata(**mid)

    @classmethod
    def _parse_hub(cls, hub: str, line: int) -> "Hub":
        ls = hub.split(maxsplit=3)
        if not 3 <= len(ls) <= 4:
            raise Exception(f"Error: Line #{line} -> malformed hub ({hub})")
        try:
            name = ls[0]
            if "-" in name:
                raise Exception(
                    f"Error: Line #{line} -> zone name '{name}' must not contain dashes"
                )
            x = int(ls[1])
            y = int(ls[2])
        except ValueError as e:
            raise Exception(f"Error: Line #{line} -> {e}")

        if len(ls) == 3:
            return Hub(name=name, x=x, y=y, metadata=HubMetadata())
        return Hub(name=name, x=x, y=y, metadata=Hub._parse_metadata(ls[-1], line))


class Map(BaseModel):
    nb_drones: int = Field(ge=1)
    start_hub: str
    end_hub: str
    hubs: dict[str, Hub]
    conns: list[Connection]

    def display(self):
        print(
            f"nb_drones: {self.nb_drones}\n\n"
            f"start_hub: {self.start_hub}\n\n"
            f"end_hub: {self.end_hub}\n\n"
        )
        for name, hub in self.hubs.items():
            print(f"{name}: {hub}\n")
        print()
        for conn in self.conns:
            print(f"{conn.h1}-{conn.h2} w/ max_link_cap={conn.max_link_cap}")
        print()

    # Parser constraints
    #   1st line MUST define the number of drones using nb_drones: <positive int>
    #   The program must be able to handle any number of drones
    #   There MUST be exactly one start_hub and one end_hub
    #   Each zone must have a UNIQUE NAME and valid integer coordinates
    #   Zone names can use ANY VALID CHARS EXCEPT DASHES AND SPACES
    #   Connections must link ONLY PREVIOUSLY DEFINED ZONES using connection: <zone1>-<zone2> [metadata]
    #   The same connection MUST NOT APPEAR MORE THAN ONCE: a-b and b-a are considered duplicates
    #   Any metadata (zone or conn) block must be syntactically valid
    #   Zone types must be one of: normal, blocked, restricted, priority. Any invalid type must raise a parsing error.
    #   Capacity values (max_drones, max_link_cap) must be positive ints.
    #   Any other parsing error must stop the program and return a clear error message indicating the line and cause

    # Implementation of dijkstra to get the minimum possible # of turns to get one drone from source to sink
    def run_dijkstra(self, reverse: bool = False) -> tuple[dict[str, int], dict[str, str | None], int]: 
       # From start, we want to visit each neighbor, set tentative values for cost to get there. 
       # Then visit new neighbors, in order of the min-heap (least cost guess)
       inf: int = 10 ** 9
       dist: dict[str, int] = {self.start_hub: 0}
       parent: dict[str, str | None] = {self.start_hub: None}
       settled: set[str] = set()

       heap: list[tuple[int, str]] = [(0, self.start_hub)]

       while heap:
           d, zone = heapq.heappop(heap)

           if zone in settled:
               continue
           settled.add(zone)

           if zone == self.end_hub:
               break

           for nb in self.hubs[zone].conns:
               if self.hubs[nb].metadata.zone == ZoneType.BLOCKED or nb in settled:
                   continue

               entered = zone if reverse else nb
               cost = 2 if self.hubs[entered].metadata.zone == ZoneType.RESTRICTED else 1
               new_d = d + cost

               if new_d < dist.get(nb, inf):
                   dist[nb] = new_d
                   parent[nb] = zone
                   heapq.heappush(heap, (new_d, nb))

       return dist, parent, dist[self.end_hub]

    @classmethod
    def _rebuild_path(cls, parent: dict[str, str | None], target: str) -> list[str]:
        path: list[str] = []
        node: str | None = target
        while node is not None:
            path.append(node)
            node = parent.get(node)
        return path[::-1]


    @classmethod
    def get_from_file(cls, path: str) -> "Map":
        try:
            with open(path, "r") as f:
                lines = f.readlines()
        except Exception:
            raise Exception(f"Failed reading file @ {path}")
        first = True
        start_hub: Hub | None = None
        end_hub: Hub | None = None
        hubs: dict[str, Hub] = {}
        conns: list[Connection] = []
        nb_drones: int | None = None
        seen_conns: dict[frozenset[str], int] = {}
        for i, line in enumerate(lines):
            if line.lstrip().startswith("#"):
                continue
            elif line.strip() == "":
                continue
            elif line.lstrip().startswith("nb_drones:") and not first:
                raise Exception(
                    f"Error: Line #{i} -> nb_drones defined not on first line."
                )
            elif line.lstrip().startswith("nb_drones:") and first:
                try:
                    nb_drones = int(line.split(":")[1].strip())
                    if nb_drones <= 0:
                        raise Exception(
                            f"Error: Line #{i} -> nb_drones must be a positive int"
                        )
                except ValueError:
                    raise Exception(
                        f"Error: Line #{i} -> Error parsing nb_drones: {line.split(':')[1]}"
                    )
            elif line.lstrip().startswith("start_hub:"):
                if start_hub:
                    raise Exception(f"Error: Line #{i} -> Multiple start_hubs defined.")
                start_hub = Hub._parse_hub(line.split(":")[1].strip(), i)
                if start_hub.name in hubs:
                    raise Exception(
                        f"Error: Line #{i} -> start_hub shares name with already existing hub {start_hub.name}"
                    )
                hubs[start_hub.name] = start_hub
            elif line.lstrip().startswith("end_hub:"):
                if end_hub:
                    raise Exception(f"Error: Line #{i} -> Multiple end_hubs defined.")
                end_hub = Hub._parse_hub(line.split(":")[1].strip(), i)
                if end_hub.name in hubs:
                    raise Exception(
                        f"Error: Line #{i} -> end_hub shares name with already existing hub {end_hub.name}"
                    )
                hubs[end_hub.name] = end_hub
            elif line.lstrip().startswith("hub:"):
                hub = Hub._parse_hub(line.split(":")[1].strip(), i)
                if hub.name in hubs:
                    raise Exception(
                        f"Error: Line #{i} -> Multiple hubs with name {hub.name} defined"
                    )
                hubs[hub.name] = hub
            elif line.lstrip().startswith("connection:"):
                conn = Connection._parse_connection(line.split(":")[1].strip(), i)
                if conn.h1 not in hubs:
                    raise Exception(
                        f"Error: Line #{i} -> Connections must link only previously defined zones, {conn.h1} has not yet been defined."
                    )
                if conn.h2 not in hubs:
                    raise Exception(
                        f"Error: Line #{i} -> Connections must link only previously defined zones, {conn.h2} has not yet been defined."
                    )
                if conn.h1 == conn.h2:
                    continue
                key = frozenset((conn.h1, conn.h2))
                if key in seen_conns:
                    raise Exception(
                        f"Error: Line #{i} -> Duplicate connection {conn.h1}-{conn.h2} "
                        f"(first defined on line #{seen_conns[key]}"
                    )
                seen_conns[key] = i
                conns.append(conn)
            else:
                raise Exception(f"Error: Line #{i} -> Invalid format ({line})")
            first = False
        for conn in conns:
            try:
                hubs[conn.h1].conns[conn.h2] = conn
                hubs[conn.h2].conns[conn.h1] = conn
            except Exception:
                raise Exception(
                    f"Error: Failed to create connection ({conn.h1}-{conn.h2})"
                )

        if nb_drones is None:
            raise Exception("Error: Missing nb_drones")
        if not start_hub or not end_hub:
            raise Exception("Error: Missing start and/or end hub")
        return Map(
            nb_drones=nb_drones,
            start_hub=start_hub.name,
            end_hub=end_hub.name,
            hubs=hubs,
            conns=conns,
        )


def main():
    try:
        mp = Map.get_from_file("./maps/challenger/01_the_impossible_dream.txt")
        mp.display()
        _, parent, cost = mp.run_dijkstra()
        shortest_path = Map._rebuild_path(parent, mp.end_hub) 
        print(f"The cost from source to sink is {cost}")
        for zone in shortest_path:
            print(f"->{zone}", end="")
    except Exception as e:
        print(e)

    print()
    try:
        print(Map.get_from_file("./maps/easy/bad_01_linear_path.txt"))
    except Exception as e:
        print(e)

    try:
        print(Map.get_from_file("./maps/easy/multiple_starts.txt"))
    except Exception as e:
        print(e)

    try:
        print(Map.get_from_file("./maps/easy/multiple_ends.txt"))
    except Exception as e:
        print(e)


def test_neg_nb_drones():
    with pytest.raises(Exception, match="nb_drones must be a positive int"):
        Map.get_from_file("./maps/easy/neg_nb_drones.txt")


def test_nbdrones_not_first():
    with pytest.raises(Exception, match="nb_drones defined not on first line"):
        Map.get_from_file("./maps/easy/bad_01_linear_path.txt")


def test_mult_starts():
    with pytest.raises(Exception, match="Multiple start_hubs"):
        Map.get_from_file("./maps/easy/multiple_starts.txt")


def test_mult_ends():
    with pytest.raises(Exception, match="Multiple end_hubs"):
        Map.get_from_file("./maps/easy/multiple_ends.txt")


if __name__ == "__main__":
    main()
