// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: http://www.viva64.com

#include <SFML/Graphics.hpp>

#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>

struct Assets
{
    sf::Font font;
};

struct Position
{
    sf::CircleShape circle;
    sf::Text id;

    inline void draw(sf::RenderWindow& window) const
    {
        window.draw(circle);
        window.draw(id);
    }
};

struct Transition
{
    sf::RectangleShape rect;
    sf::Text id;

    inline void draw(sf::RenderWindow& window) const
    {
        window.draw(rect);
        window.draw(id);
    }
};

struct Edge
{
    sf::Vertex line[2];
    sf::Vertex triangle[3];

    inline void draw(sf::RenderWindow& window) const
    {
        window.draw(line, 2, sf::Lines);
        window.draw(triangle, 3, sf::Triangles);
    }
};

struct Marker
{
    sf::CircleShape circle;
    sf::Text id;

    inline void draw(sf::RenderWindow& window) const
    {
        window.draw(circle);
        window.draw(id);
    }

    inline void die()
    {
        circle.setRadius(0);
        id.setString("");
    }

    inline void block()
    {
        circle.setFillColor(sf::Color::Red);
    }

    inline void move(sf::Vector2f pos)
    {
        pos.x += 10;
        pos.y += 10;

        circle.setPosition(pos);
        id.setPosition(pos);
    }
};

void AddPosition(Assets& assets, std::vector<Position>& positions, sf::Vector2f pos);
void AddTransition(Assets& assets, std::vector<Transition>& transitions, sf::Vector2f pos);
void AddEdge(std::vector<Edge>& edges, sf::Vector2f from, sf::Vector2f to);
void AddMarker(Assets& assets, std::vector<Marker>& markers, sf::Vector2f pos);
std::vector<Position> CreatePositions(Assets& assets);
std::vector<Transition> CreateTransitions(Assets& assets);
std::vector<Edge> CreateEdges();
std::vector<Marker> CreateMarkers(Assets& assets);

auto main() -> int
{
    Assets assets;
    assets.font.loadFromFile("didot.ttf");

    std::ifstream ifstr("../PetriNet/output.txt");

    const auto positions = CreatePositions(assets);
    const auto transitions = CreateTransitions(assets);
    const auto edges = CreateEdges();
    auto markers = CreateMarkers(assets);

    sf::RenderWindow window(sf::VideoMode(800, 600), "PetriNet Visualization");
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            else if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Space)
            {
                std::string str;
                std::getline(ifstr, str);
                if (!str.empty())
                {
                    size_t markerIdx{};
                    std::stringstream(str.substr(str.find_first_of('<') + 1)) >> markerIdx;

                    str = str.substr(str.find_first_of('[') + 1);
                    if (str.front() == '-')
                        markers.at(markerIdx).die();
                    else if (str.front() == 'X')
                        markers.at(markerIdx).block();
                    else
                    {
                        char dummy{};
                        size_t positionIdx;
                        std::stringstream(str) >> dummy >> dummy >> dummy >> positionIdx;

                        markers.at(markerIdx).move(positions.at(positionIdx).circle.getPosition());
                    }
                }
            }
        }

        

        window.clear(sf::Color::White);

        for (const auto& obj : edges) obj.draw(window);
        for (const auto& obj : positions)   obj.draw(window);
        for (const auto& obj : transitions) obj.draw(window);

        for (const auto& obj : markers) obj.draw(window);

        window.display();
    }

    return 0;
}

void AddPosition(Assets& assets, std::vector<Position>& positions, sf::Vector2f pos)
{
    static constexpr auto R{ 25.f };

    pos.y -= R;

    sf::Text text;
    text.setFont(assets.font);
    text.setString(std::to_string(std::size(positions)));
    text.setPosition(pos);
    text.setFillColor(sf::Color::Blue);
    text.setCharacterSize(25);

    pos.x -= R;

    sf::CircleShape circle(R);
    circle.setPosition(pos);
    circle.setOutlineColor(sf::Color::Black);
    circle.setOutlineThickness(2.f);

    positions.emplace_back(circle, text);
}

void AddTransition(Assets& assets, std::vector<Transition>& transitions, sf::Vector2f pos)
{
    static constexpr auto A{ 50.f };

    pos.y -= A / 2;

    sf::Text text;
    text.setFont(assets.font);
    text.setString(std::to_string(std::size(transitions)));
    text.setPosition(pos);
    text.setFillColor(sf::Color::Red);
    text.setCharacterSize(25);

    pos.x -= A / 2;

    sf::RectangleShape rect({ A, A });
    rect.setPosition(pos);
    rect.setOutlineColor(sf::Color::Black);
    rect.setOutlineThickness(2.f);

    transitions.emplace_back(rect, text);
}

void AddEdge(std::vector<Edge>& edges, sf::Vector2f from, sf::Vector2f to)
{
    Edge e{ from, to };
    e.line[0].color = sf::Color::Black;
    e.line[1].color = sf::Color::Black;

    e.triangle[0].position = { from.x + (to.x - from.x) / 2.f, from.y + (to.y - from.y) / 2.f };
    e.triangle[0].color = sf::Color::Black;

    auto val = std::sqrt(2.f) / 2.f * .2f;
    e.triangle[1].position =
    { 
        e.triangle[0].position.x + ((from.x - e.triangle[0].position.x) + (from.y - e.triangle[0].position.y)) * val,
        e.triangle[0].position.y + ((from.y - e.triangle[0].position.y) - (from.x - e.triangle[0].position.x)) * val
    };
    e.triangle[1].color = sf::Color::Black;

    e.triangle[2].position =
    {
        e.triangle[0].position.x + ((from.x - e.triangle[0].position.x) - (from.y - e.triangle[0].position.y)) * val,
        e.triangle[0].position.y + ((from.y - e.triangle[0].position.y) + (from.x - e.triangle[0].position.x)) * val
    };
    e.triangle[2].color = sf::Color::Black;

    edges.push_back(e);
}

void AddMarker(Assets& assets, std::vector<Marker>& markers, sf::Vector2f pos)
{
    static constexpr auto R{ 5.f };
    pos.y -= R;
    pos.x -= R;

    sf::Text text;
    text.setFont(assets.font);
    text.setString(std::to_string(std::size(markers)));
    text.setPosition(pos);
    text.setFillColor(sf::Color::Magenta);
    text.setCharacterSize(15);
    text.setStyle(sf::Text::Bold);

    sf::CircleShape circle(R);
    circle.setPosition(pos);
    circle.setOutlineColor(sf::Color::Black);
    circle.setFillColor(sf::Color::Black);

    markers.emplace_back(circle, text);
}

std::vector<Position> CreatePositions(Assets& assets)
{
    std::vector<Position> positions;
    AddPosition(assets, positions, { 150.f, 300.f });

    AddPosition(assets, positions, { 350.f, 200.f });
    AddPosition(assets, positions, { 350.f, 400.f });

    AddPosition(assets, positions, { 550.f, 200.f });
    AddPosition(assets, positions, { 550.f, 400.f });

    AddPosition(assets, positions, { 550.f, 500.f });

    return positions;
}

std::vector<Transition> CreateTransitions(Assets& assets)
{
    std::vector<Transition> transitions;

    AddTransition(assets, transitions, { 250.f, 200.f });
    AddTransition(assets, transitions, { 250.f, 400.f });

    AddTransition(assets, transitions, { 450.f, 100.f });
    AddTransition(assets, transitions, { 450.f, 200.f });
    AddTransition(assets, transitions, { 450.f, 400.f });
    AddTransition(assets, transitions, { 450.f, 500.f });

    AddTransition(assets, transitions, { 700.f, 300.f });

    return transitions;
}

std::vector<Edge> CreateEdges()
{
    std::vector<Edge> edges;
    AddEdge(edges, { 150.f, 300.f }, { 250.f, 200.f });
    AddEdge(edges, { 150.f, 300.f }, { 250.f, 400.f });

    AddEdge(edges, { 250.f, 200.f }, { 350.f, 200.f });
    AddEdge(edges, { 250.f, 400.f }, { 350.f, 400.f });

    AddEdge(edges, { 350.f, 200.f }, { 450.f, 200.f });
    AddEdge(edges, { 450.f, 200.f }, { 550.f, 400.f });
    AddEdge(edges, { 350.f, 400.f }, { 450.f, 400.f });
    AddEdge(edges, { 350.f, 400.f }, { 450.f, 200.f });

    AddEdge(edges, { 450.f, 200.f }, { 550.f, 200.f });
    AddEdge(edges, { 450.f, 400.f }, { 550.f, 400.f });


    AddEdge(edges, { 550.f, 200.f }, { 450.f, 100.f });
    AddEdge(edges, { 450.f, 100.f }, { 350.f, 200.f });

    AddEdge(edges, { 550.f, 400.f }, { 700.f, 300.f });
    AddEdge(edges, { 550.f, 400.f }, { 450.f, 500.f });
    AddEdge(edges, { 450.f, 500.f }, { 350.f, 400.f });
    AddEdge(edges, { 450.f, 500.f }, { 550.f, 500.f });

    return edges;
}

std::vector<Marker> CreateMarkers(Assets& assets)
{
    std::vector<Marker> markers;
    AddMarker(assets, markers, { 135.f, 290.f });
    AddMarker(assets, markers, { 135.f, 310.f });

    return markers;
}
