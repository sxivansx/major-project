// Room map (handheld 2D mapping): the latest "room" message and the latest
// kind-6 occupancy grid, plus start / stop.

import { create } from "zustand";
import { api, errorText } from "../api/rest";
import type { RoomFrame, RoomState } from "../api/types";
import { clientLog } from "./store";

export interface RoomStore {
  room: RoomState | null;
  grid: RoomFrame | null;
  supported: boolean | null; // null until GET /api/room answered
  error: string | null;
}

export const useRoom = create<RoomStore>(() => ({ room: null, grid: null, supported: null, error: null }));

export function roomSetState(r: RoomState): void {
  useRoom.setState({ room: r, supported: true });
}

export function roomSetGrid(g: RoomFrame): void {
  useRoom.setState({ grid: g, supported: true });
}

// After a page load: the state and, if there is a map, its grid.
export async function loadRoom(): Promise<void> {
  try {
    const r = await api.room();
    useRoom.setState({ room: r, supported: true, error: null });
    if (r.grid && r.grid.w > 0 && r.grid.h > 0) {
      try {
        useRoom.setState({ grid: await api.roomGrid() });
      } catch {
        // no grid yet
      }
    }
  } catch (e) {
    useRoom.setState({ supported: false, error: errorText(e) });
  }
}

export async function roomStart(cell_m?: number): Promise<void> {
  try {
    useRoom.setState({ grid: null, error: null });
    await api.roomStart(cell_m);
    await loadRoom();
    clientLog("info", "room mapping started");
  } catch (e) {
    useRoom.setState({ error: errorText(e) });
    clientLog("error", `room/start: ${errorText(e)}`);
    throw e;
  }
}

export async function roomStop(): Promise<void> {
  try {
    await api.roomStop();
    await loadRoom();
  } catch (e) {
    useRoom.setState({ error: errorText(e) });
    clientLog("error", `room/stop: ${errorText(e)}`);
    throw e;
  }
}
