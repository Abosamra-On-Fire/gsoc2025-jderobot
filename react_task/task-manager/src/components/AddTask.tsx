import React, { useState } from 'react';
import DatePicker from 'react-datepicker';
import 'react-datepicker/dist/react-datepicker.css';
import { useAppDispatch } from '../hooks';
import { addTask } from '../store/taskSlice';
import styled from 'styled-components';
import { Priority, Category } from '../types/task';

const Form = styled.form`
  display: flex;
  flex-direction: column;
  gap: 1rem;
  margin-bottom: 2rem;
`;

const Input = styled.input`
  padding: 0.5rem;
  border: 1px solid #ddd;
  border-radius: 4px;
`;

const Select = styled.select`
  padding: 0.5rem;
  border: 1px solid #ddd;
  border-radius: 4px;
`;

const Button = styled.button`
  padding: 0.5rem 1rem;
  background-color: #007bff;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  &:hover {
    background-color: #0056b3;
  }
`;

const AddTask: React.FC = () => {
  const dispatch = useAppDispatch();
  const [title, setTitle] = useState('');
  const [priority, setPriority] = useState<Priority>('medium');
  const [category, setCategory] = useState<Category>('personal');
  const [dueDate, setDueDate] = useState<Date | null>(null);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (!title.trim()) return;

    dispatch(
      addTask({
        title: title.trim(),
        completed: false,
        priority,
        category,
        dueDate: dueDate || undefined,
      })
    );

    setTitle('');
    setPriority('medium');
    setCategory('personal');
    setDueDate(null);
  };

  return (
    <Form onSubmit={handleSubmit}>
      <Input
        type="text"
        placeholder="Task title"
        value={title}
        onChange={(e) => setTitle(e.target.value)}
        required
      />
      <Select
        value={priority}
        onChange={(e) => setPriority(e.target.value as Priority)}
      >
        <option value="low">Low Priority</option>
        <option value="medium">Medium Priority</option>
        <option value="high">High Priority</option>
      </Select>
      <Select
        value={category}
        onChange={(e) => setCategory(e.target.value as Category)}
      >
        <option value="personal">Personal</option>
        <option value="work">Work</option>
        <option value="groceries">Groceries</option>
        <option value="other">Other</option>
      </Select>
      <DatePicker
        selected={dueDate}
        onChange={(date) => setDueDate(date)}
        placeholderText="Select due date (optional)"
        dateFormat="MM/dd/yyyy"
        minDate={new Date()}
      />
      <Button type="submit">Add Task</Button>
    </Form>
  );
};

export default AddTask; 