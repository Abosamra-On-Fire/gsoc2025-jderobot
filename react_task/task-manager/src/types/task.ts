export type Priority = 'low' | 'medium' | 'high';
export type Category = 'personal' | 'work' | 'groceries' | 'other';

export interface Task {
  id: string;
  title: string;
  completed: boolean;
  priority: Priority;
  category: Category;
  dueDate?: Date;
  createdAt: Date;
}

export interface TaskState {
  tasks: Task[];
  filter: {
    status: 'all' | 'completed' | 'incomplete';
    category: Category | 'all';
    priority: Priority | 'all';
  };
  searchQuery: string;
} 