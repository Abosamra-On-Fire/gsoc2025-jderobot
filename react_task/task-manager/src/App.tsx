import React from 'react';
import { Provider } from 'react-redux';
import { store } from './store';
import styled from 'styled-components';
import AddTask from './components/AddTask';
import TaskList from './components/TaskList';

const AppContainer = styled.div`
  min-height: 100vh;
  background-color: #f8f9fa;
`;

const Header = styled.header`
  background-color: #343a40;
  color: white;
  padding: 1rem;
  text-align: center;
`;

const Title = styled.h1`
  margin: 0;
  font-size: 2rem;
`;

const Main = styled.main`
  max-width: 800px;
  margin: 0 auto;
  padding: 2rem;
`;

function App() {
  return (
    <Provider store={store}>
      <AppContainer>
        <Header>
          <Title>Task Manager</Title>
        </Header>
        <Main>
          <AddTask />
          <TaskList />
        </Main>
      </AppContainer>
    </Provider>
  );
}

export default App;
