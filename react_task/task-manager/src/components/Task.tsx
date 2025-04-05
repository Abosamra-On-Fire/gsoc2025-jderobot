import React, { useRef } from 'react';
import { useDrag, useDrop } from 'react-dnd';
import { Task as TaskType } from '../types/task';
import styled from 'styled-components';
import { FaTrash, FaCheck } from 'react-icons/fa';

interface TaskProps {
  task: TaskType;
  index: number;
  onToggle: (id: string) => void;
  onDelete: (id: string) => void;
  onMove: (fromIndex: number, toIndex: number) => void;
}

const TaskContainer = styled.div<{ isDragging: boolean }>`
  display: flex;
  align-items: center;
  padding: 1rem;
  margin: 0.5rem 0;
  background-color: ${props => props.isDragging ? '#f0f0f0' : 'white'};
  border: 1px solid #ddd;
  border-radius: 4px;
  cursor: move;
  transition: background-color 0.2s;
`;

const TaskContent = styled.div`
  flex: 1;
  display: flex;
  align-items: center;
  gap: 1rem;
`;

const TaskTitle = styled.span<{ completed: boolean }>`
  text-decoration: ${props => props.completed ? 'line-through' : 'none'};
  color: ${props => props.completed ? '#888' : 'inherit'};
`;

const PriorityIndicator = styled.div<{ priority: string }>`
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background-color: ${props => {
    switch (props.priority) {
      case 'high':
        return '#ff4444';
      case 'medium':
        return '#ffbb33';
      case 'low':
        return '#00C851';
      default:
        return '#888';
    }
  }};
`;

const CategoryBadge = styled.span`
  padding: 0.25rem 0.5rem;
  background-color: #e9ecef;
  border-radius: 4px;
  font-size: 0.8rem;
`;

const DueDate = styled.span`
  font-size: 0.8rem;
  color: #666;
`;

const IconButton = styled.button`
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 0.5rem;
  border: none;
  background: none;
  cursor: pointer;
  color: inherit;
  svg {
    width: 1.2em;
    height: 1.2em;
  }
`;

const IconWrapper = styled.div<{ color?: string }>`
  color: ${props => props.color || 'inherit'};
  display: flex;
  align-items: center;
  justify-content: center;
`;

const Task: React.FC<TaskProps> = ({ task, index, onToggle, onDelete, onMove }) => {
  const ref = useRef<HTMLDivElement>(null);
  
  const [{ isDragging }, drag] = useDrag({
    type: 'TASK',
    item: { index },
    collect: (monitor) => ({
      isDragging: monitor.isDragging(),
    }),
  });

  const [, drop] = useDrop({
    accept: 'TASK',
    hover: (item: { index: number }) => {
      if (item.index !== index) {
        onMove(item.index, index);
        item.index = index;
      }
    },
  });

  drag(drop(ref));

  return (
    <TaskContainer
      ref={ref}
      isDragging={isDragging}
    >
      <TaskContent>
        <IconButton onClick={() => onToggle(task.id)}>
          <IconWrapper color={task.completed ? '#00C851' : '#ddd'}>
            {FaCheck({})}
          </IconWrapper>
        </IconButton>
        <PriorityIndicator priority={task.priority} />
        <TaskTitle completed={task.completed}>{task.title}</TaskTitle>
        <CategoryBadge>{task.category}</CategoryBadge>
        {task.dueDate && (
          <DueDate>
            Due: {new Date(task.dueDate).toLocaleDateString()}
          </DueDate>
        )}
      </TaskContent>
      <IconButton onClick={() => onDelete(task.id)}>
        <IconWrapper color="#ff4444">
          {FaTrash({})}
        </IconWrapper>
      </IconButton>
    </TaskContainer>
  );
};

export default Task; 