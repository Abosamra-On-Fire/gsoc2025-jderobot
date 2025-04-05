import React from 'react';
import { DndProvider } from 'react-dnd';
import { HTML5Backend } from 'react-dnd-html5-backend';
import { useAppSelector, useAppDispatch } from '../hooks';
import Task from './Task';
import {
  toggleTask,
  deleteTask,
  reorderTasks,
  setFilterStatus,
  setFilterCategory,
  setFilterPriority,
  setSearchQuery,
} from '../store/taskSlice';
import styled from 'styled-components';
import { Priority, Category } from '../types/task';

const Container = styled.div`
  max-width: 800px;
  margin: 0 auto;
  padding: 2rem;
`;

const Filters = styled.div`
  display: flex;
  gap: 1rem;
  margin-bottom: 1rem;
`;

const SearchInput = styled.input`
  width: 100%;
  padding: 0.5rem;
  margin-bottom: 1rem;
  border: 1px solid #ddd;
  border-radius: 4px;
`;

const Select = styled.select`
  padding: 0.5rem;
  border: 1px solid #ddd;
  border-radius: 4px;
`;

const TaskList: React.FC = () => {
  const dispatch = useAppDispatch();
  const { tasks, filter, searchQuery } = useAppSelector((state) => state.tasks);

  const filteredTasks = tasks
    .filter((task) => {
      if (filter.status === 'completed') return task.completed;
      if (filter.status === 'incomplete') return !task.completed;
      return true;
    })
    .filter((task) => {
      if (filter.category === 'all') return true;
      return task.category === filter.category;
    })
    .filter((task) => {
      if (filter.priority === 'all') return true;
      return task.priority === filter.priority;
    })
    .filter((task) =>
      task.title.toLowerCase().includes(searchQuery.toLowerCase())
    );

  const handleMoveTask = (fromIndex: number, toIndex: number) => {
    dispatch(reorderTasks({ fromIndex, toIndex }));
  };

  return (
    <Container>
      <SearchInput
        type="text"
        placeholder="Search tasks..."
        value={searchQuery}
        onChange={(e) => dispatch(setSearchQuery(e.target.value))}
      />
      <Filters>
        <Select
          value={filter.status}
          onChange={(e) => dispatch(setFilterStatus(e.target.value as any))}
        >
          <option value="all">All Tasks</option>
          <option value="completed">Completed</option>
          <option value="incomplete">Incomplete</option>
        </Select>
        <Select
          value={filter.category}
          onChange={(e) => dispatch(setFilterCategory(e.target.value as Category))}
        >
          <option value="all">All Categories</option>
          <option value="personal">Personal</option>
          <option value="work">Work</option>
          <option value="groceries">Groceries</option>
          <option value="other">Other</option>
        </Select>
        <Select
          value={filter.priority}
          onChange={(e) => dispatch(setFilterPriority(e.target.value as Priority))}
        >
          <option value="all">All Priorities</option>
          <option value="high">High</option>
          <option value="medium">Medium</option>
          <option value="low">Low</option>
        </Select>
      </Filters>
      <DndProvider backend={HTML5Backend}>
        {filteredTasks.map((task, index) => (
          <Task
            key={task.id}
            task={task}
            index={index}
            onToggle={(id) => dispatch(toggleTask(id))}
            onDelete={(id) => dispatch(deleteTask(id))}
            onMove={handleMoveTask}
          />
        ))}
      </DndProvider>
    </Container>
  );
};

export default TaskList; 