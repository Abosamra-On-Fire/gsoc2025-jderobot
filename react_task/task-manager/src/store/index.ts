import { configureStore } from '@reduxjs/toolkit';
import taskReducer from './taskSlice';
import { loadState, saveState } from '../utils/storage';

const persistedState = loadState();

export const store = configureStore({
  reducer: {
    tasks: taskReducer,
  },
  preloadedState: persistedState ? { tasks: persistedState } : undefined,
});

store.subscribe(() => {
  saveState(store.getState().tasks);
});

export type RootState = ReturnType<typeof store.getState>;
export type AppDispatch = typeof store.dispatch; 