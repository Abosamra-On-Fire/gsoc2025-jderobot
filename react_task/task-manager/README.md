# Task Manager Application

A modern task management application built with React, Redux, and TypeScript. This application allows users to manage their tasks with features like filtering, categorization, priority setting, and drag-and-drop reordering.

## Features

- Add, edit, and delete tasks
- Set task priority (high, medium, low)
- Categorize tasks (personal, work, groceries, other)
- Set due dates for tasks
- Filter tasks by status, category, and priority
- Search tasks by title
- Drag and drop to reorder tasks
- Responsive design

## Prerequisites

- Node.js (v14 or higher)
- npm (v6 or higher)

## Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd task-manager
```

2. Install dependencies:
```bash
npm install
```

3. Start the development server:
```bash
npm start
```

The application will open in your default browser at `http://localhost:3000`.

## Usage

### Adding a Task
1. Enter the task title in the input field
2. Select a priority level (low, medium, high)
3. Choose a category (personal, work, groceries, other)
4. Optionally set a due date
5. Click "Add Task"

### Managing Tasks
- Click the checkmark to toggle task completion
- Click the trash icon to delete a task
- Drag and drop tasks to reorder them
- Use the filters to view specific tasks
- Use the search bar to find tasks by title

### Filtering Tasks
- Use the status filter to show all, completed, or incomplete tasks
- Use the category filter to show tasks from specific categories
- Use the priority filter to show tasks with specific priority levels

## Technologies Used

- React
- Redux Toolkit
- TypeScript
- Styled Components
- React DnD (for drag and drop)
- React DatePicker
- React Icons

## Project Structure

```
src/
  ├── components/     # React components
  ├── store/         # Redux store and slices
  ├── types/         # TypeScript type definitions
  ├── hooks/         # Custom React hooks
  └── utils/         # Utility functions
```

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.
