import { createSlice, PayloadAction } from '@reduxjs/toolkit';
import { Task, TaskState, Priority, Category } from '../types/task';

const initialState: TaskState = {
  tasks: [],
  filter: {
    status: 'all',
    category: 'all',
    priority: 'all',
  },
  searchQuery: '',
};

const taskSlice = createSlice({
  name: 'tasks',
  initialState,
  reducers: {
    addTask: (state, action: PayloadAction<Omit<Task, 'id' | 'createdAt'>>) => {
      const newTask: Task = {
        ...action.payload,
        id: Date.now().toString(),
        createdAt: new Date(),
      };
      state.tasks.push(newTask);
    },
    toggleTask: (state, action: PayloadAction<string>) => {
      const task = state.tasks.find((t) => t.id === action.payload);
      if (task) {
        task.completed = !task.completed;
      }
    },
    deleteTask: (state, action: PayloadAction<string>) => {
      state.tasks = state.tasks.filter((task) => task.id !== action.payload);
    },
    setFilterStatus: (state, action: PayloadAction<TaskState['filter']['status']>) => {
      state.filter.status = action.payload;
    },
    setFilterCategory: (state, action: PayloadAction<TaskState['filter']['category']>) => {
      state.filter.category = action.payload;
    },
    setFilterPriority: (state, action: PayloadAction<TaskState['filter']['priority']>) => {
      state.filter.priority = action.payload;
    },
    setSearchQuery: (state, action: PayloadAction<string>) => {
      state.searchQuery = action.payload;
    },
    reorderTasks: (state, action: PayloadAction<{ fromIndex: number; toIndex: number }>) => {
      const { fromIndex, toIndex } = action.payload;
      const [movedTask] = state.tasks.splice(fromIndex, 1);
      state.tasks.splice(toIndex, 0, movedTask);
    },
  },
});

export const {
  addTask,
  toggleTask,
  deleteTask,
  setFilterStatus,
  setFilterCategory,
  setFilterPriority,
  setSearchQuery,
  reorderTasks,
} = taskSlice.actions;

export default taskSlice.reducer; 